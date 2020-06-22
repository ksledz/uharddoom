#include <linux/module.h>
#include <linux/pci.h>
#include <linux/cdev.h>
#include <linux/anon_inodes.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/file.h>
#include <linux/kref.h>
#include <linux/interrupt.h>

#include "uharddoom.h"
#include "udoomdev.h"
#include "udoomfw.h"

#define UHARDDOOM_MAX_DEVICES 256
#define UHARDDOOM_NUM_JOBS 16
#include <linux/kernel.h>
#include <linux/err.h>

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("UHardDoom device driver for Advanced Operating Systems class");

struct uharddoom_map {
	struct uharddoom_context* context;
	uint32_t virtual;
	struct uharddoom_buffer* buffer;
	struct list_head lh;
	struct file* file;

};

struct uharddoom_page {
        void *page;
        dma_addr_t addr;
};

struct uharddoom_buffer {
	struct uharddoom_page *pages;
	struct uharddoom_device *device;
	uint32_t pages_nr;	
};

struct uharddoom_job {
	struct list_head lh;
	struct uharddoom_context *ctx;
	uint32_t size;
	uint32_t addr;
};

struct uharddoom_context {
	struct uharddoom_device *dev;
	int pending_jobs;
	wait_queue_head_t wq;
	uint32_t* page_directory;
	dma_addr_t page_directory_dma;
	uint32_t** page_tables;
	dma_addr_t* page_table_dma;
	struct list_head maps;
	bool error;
	struct mutex mutex;
};

struct uharddoom_device {
	struct pci_dev *pdev;
	struct cdev cdev;
	int idx;
	struct device *dev;
	void __iomem *bar;
	spinlock_t slock;
	struct list_head jobs_free;
	struct list_head jobs_running;
	wait_queue_head_t free_wq;
	wait_queue_head_t idle_wq;
};

static dev_t uharddoom_devno;
static struct uharddoom_device *uharddoom_devices[UHARDDOOM_MAX_DEVICES];
static DEFINE_MUTEX(uharddoom_devices_lock);
static struct class uharddoom_class = {
	.name = "uharddoom",
	.owner = THIS_MODULE,
};


/* Hardware handling. */

static inline void uharddoom_iow(struct uharddoom_device *dev, uint32_t reg, uint32_t val)
{
	iowrite32(val, dev->bar + reg);
//	printk(KERN_ALERT "uharddoom %03x <- %08x\n", reg, val);
}

static inline uint32_t uharddoom_ior(struct uharddoom_device *dev, uint32_t reg)
{
	uint32_t res = ioread32(dev->bar + reg);
//	printk(KERN_ALERT "uharddoom %03x -> %08x\n", reg, res);
	return res;
}

/* IRQ handler.  */

static irqreturn_t uharddoom_isr(int irq, void *opaque)
{
	struct uharddoom_device *dev = opaque;
	unsigned long flags;
	uint32_t istatus;
	struct uharddoom_job *job;
	spin_lock_irqsave(&dev->slock, flags);
//	printk(KERN_ALERT "uharddoom isr\n");
	istatus = uharddoom_ior(dev, UHARDDOOM_INTR) & uharddoom_ior(dev, UHARDDOOM_INTR_ENABLE);
	if (istatus) {
		uharddoom_iow(dev, UHARDDOOM_INTR, istatus);
		BUG_ON(list_empty(&dev->jobs_running));
		job = list_entry(dev->jobs_running.next, struct uharddoom_job, lh);
		list_del(&job->lh);
		job->ctx->pending_jobs--;
		wake_up(&job->ctx->wq);
		if (istatus != UHARDDOOM_INTR_JOB_DONE) {
			job->ctx->error = true;
			uharddoom_iow(dev, UHARDDOOM_ENABLE, 0);
			
			uharddoom_iow(dev, UHARDDOOM_RESET, 0x7f7ffffe);
			uharddoom_iow(dev, UHARDDOOM_INTR, 0xff33);

			uharddoom_iow(dev, UHARDDOOM_ENABLE, UHARDDOOM_ENABLE_ALL & ~UHARDDOOM_ENABLE_BATCH);	
		}
		job->ctx = 0; // ?
		list_add(&job->lh, &dev->jobs_free);
		wake_up(&dev->free_wq);
		if (list_empty(&dev->jobs_running)) {
			/* No more jobs to run.  */
			wake_up(&dev->idle_wq);
		} else {
			/* Run the next job. */
			job = list_entry(dev->jobs_running.next, struct uharddoom_job, lh);
		
			uharddoom_iow(dev, UHARDDOOM_JOB_PDP, (job->ctx->page_directory_dma) >> 12);
			uharddoom_iow(dev, UHARDDOOM_JOB_CMD_PTR, job->addr);
			uharddoom_iow(dev, UHARDDOOM_JOB_CMD_SIZE, job->size);
			uharddoom_iow(dev, UHARDDOOM_JOB_TRIGGER, 1);
		}
	}
	spin_unlock_irqrestore(&dev->slock, flags);
	return IRQ_RETVAL(istatus);
}


static int uharddoom_open(struct inode *inode, struct file *file)
{
	struct uharddoom_device *dev = container_of(inode->i_cdev, struct uharddoom_device, cdev);
	struct uharddoom_context *ctx = kzalloc(sizeof *ctx, GFP_KERNEL);
	if (!ctx)	
		return -ENOMEM;
	mutex_init(&ctx->mutex);
	INIT_LIST_HEAD(&ctx->maps);
	ctx->dev = dev;
	ctx->error = false;
	init_waitqueue_head(&ctx->wq);
	ctx->pending_jobs = 0;
	file->private_data = ctx;
	/* alloc page directory full of zeros*/
	dma_addr_t dma_handle;
	//TODO sprawdzanie bledow do alloc x3
	uint32_t* pd = dma_alloc_coherent(&dev->pdev->dev, 4096,&dma_handle, GFP_KERNEL | __GFP_ZERO);
	ctx->page_directory = pd;
	ctx->page_directory_dma = dma_handle;
	ctx->page_tables = kzalloc(sizeof(*ctx->page_tables)*1024,GFP_KERNEL);
        ctx->page_table_dma = kzalloc(sizeof(*ctx->page_table_dma)*1024, GFP_KERNEL);	
	return nonseekable_open(inode, file);
}
long ioctl_unmap(struct uharddoom_context* ctx, uint32_t addr);
static int uharddoom_release(struct inode *inode, struct file *file)
{
	struct uharddoom_context *ctx = file->private_data;
	struct uharddoom_device *dev = ctx->dev;
	unsigned long flags;
	spin_lock_irqsave(&dev->slock, flags);
	while (ctx->pending_jobs) {
		spin_unlock_irqrestore(&dev->slock, flags);
		wait_event(ctx->wq, !ctx->pending_jobs);
		spin_lock_irqsave(&dev->slock, flags);
	}
	spin_unlock_irqrestore(&dev->slock, flags);
	while(!list_empty(&ctx->maps)) {
		ioctl_unmap(ctx, list_first_entry(&ctx->maps, struct uharddoom_map, lh)->virtual);
	}
	int i;	
	for (i = 0; i < 1024; i++) {
		if (ctx->page_tables[i]) {
			dma_free_coherent(&ctx->dev->pdev->dev, PAGE_SIZE, ctx->page_tables[i], ctx->page_table_dma[i]);
		}
	}
	dma_free_coherent(&ctx->dev->pdev->dev, PAGE_SIZE, ctx->page_directory, ctx->page_directory_dma); 
	kfree(ctx);
	return 0;

}


static vm_fault_t buffer_fault(struct vm_fault *vmf)
{
	struct uharddoom_buffer *buffer = vmf->vma->vm_private_data;
       if (vmf->pgoff >= buffer->pages_nr)
                return VM_FAULT_SIGBUS;

	struct page *page;
	int i = vmf->pgoff;
	page = virt_to_page(buffer->pages[i].page);
	get_page(page);
	vmf->page = page;
	return 0;
}

static const struct vm_operations_struct buffer_vm = {
	.fault = buffer_fault,
};
static int buffer_mmap(struct file *file, struct vm_area_struct *vma)
{

//	vma->vm_flags |= VM_IO | VM_DONTDUMP | VM_DONTEXPAND;
	vma->vm_ops = &buffer_vm;
	vma->vm_private_data = file->private_data;

	return 0;
}

static int buffer_release(struct inode *inode, struct file *file)
{
        struct uharddoom_buffer *buf = file->private_data;
	int i;
	for (i = 0; i < buf->pages_nr; i++) {
		dma_free_coherent(&buf->device->pdev->dev, PAGE_SIZE, buf->pages[i].page, buf->pages[i].addr);
	}
	kfree(buf->pages);
	kfree(buf);
	return 0;
}
const struct file_operations uharddoom_buf_ops = {
        .owner = THIS_MODULE,
        .release = buffer_release,
	.mmap = buffer_mmap,

};
int ioctl_create(uint32_t size, struct uharddoom_device *dev) {
	if (size == 0) return -EINVAL;
	struct uharddoom_buffer *buffer;
	buffer = kmalloc(sizeof(struct uharddoom_buffer), GFP_KERNEL);
	buffer->pages_nr = (size % PAGE_SIZE == 0) ? (size / PAGE_SIZE) : (size / PAGE_SIZE) + 1;
        buffer->pages = kmalloc(buffer->pages_nr * sizeof(struct uharddoom_page), GFP_KERNEL);
	buffer->device = dev;

	int i;
	for (i = 0; i < buffer->pages_nr; i++) {
		buffer->pages[i].page = dma_alloc_coherent(&dev->pdev->dev, PAGE_SIZE, &buffer->pages[i].addr, GFP_KERNEL | __GFP_ZERO);
	}
	return anon_inode_getfd("uharddoom buffer", &uharddoom_buf_ops, buffer, O_RDWR | O_CLOEXEC);
}

/* mapuje bufor do przestrzeni adresowej urządzenia powiązanej z obecnym kontekstem. Parametrami tego wywołania są deskryptor pliku odnoszący się do mapowanego bufora, oraz tryb mapowania (0 — do odczytu i zapisu, 1 — tylko do odczytu). Sterownik powinien sam znaleźć wolny adres wirtualny w obecnym kontekście. Wynikiem wywołania jest przydzielony adres wirtualny. W razie braku możliwości zmapowania bufora przez brak wolnej przestrzeni adresowej (bądź jej nadmierną fragmentację), należy zwrócić błąd ENOMEM. */

long allocator(struct uharddoom_context* ctx,struct uharddoom_buffer *buf, 
		struct file* file) {
	struct uharddoom_map* i;
	uint32_t prev = 0;
	bool alloced = false;
	list_for_each_entry(i, &ctx->maps, lh) {
		uint32_t curr = i->virtual;
		if (curr - prev >= buf->pages_nr * PAGE_SIZE) {
			// alokujemy
			struct uharddoom_map* new;
			new = kmalloc(sizeof(struct uharddoom_map), GFP_KERNEL);
			new->context = ctx;
			new->virtual = prev;
			new->buffer = buf;
			list_add_tail(&new->lh, &i->lh);
			new->file = get_file(file);	
			alloced = true;
			return prev;
		}
		prev = i->virtual + (i->buffer->pages_nr * PAGE_SIZE);
		if (!prev) return -ENOMEM;
	
	}
	if ((1ull << 32) - prev < buf->pages_nr * PAGE_SIZE) {
		return -ENOMEM;
	} else {
		struct uharddoom_map* new;
		new = kmalloc(sizeof(struct uharddoom_map), GFP_KERNEL);
		new->context = ctx;
		new->virtual = prev;
		new->buffer = buf;
		new->file = get_file(file);
		list_add_tail(&new->lh, &ctx->maps);
		return prev;
	}
}
long ioctl_map(struct uharddoom_context* ctx, unsigned int fd, unsigned int rdonly) {

	struct fd file_desc = fdget(fd);
	struct file* file = file_desc.file;
	if (file->f_op != &uharddoom_buf_ops) {
		fdput(file_desc);
		return -EINVAL;
	}	
	struct uharddoom_buffer* buffer = file->private_data;
	if (buffer->device != ctx->dev) {
		fdput(file_desc);
		return -EXDEV;
	}
	long res = allocator(ctx, buffer, file);
	fdput(file_desc);
	if (IS_ERR_VALUE (res)) {
		return res;
	}
	int i;
	for (i = 0; i < buffer->pages_nr; i++) {	
		uint32_t address = res + i * PAGE_SIZE;
		int index_pd = (address >> 22) & 0x3ff;
		if (!ctx->page_tables[index_pd]) {
			// sprawdzanie bledow 
			ctx->page_tables[index_pd] = dma_alloc_coherent(&buffer->device->pdev->dev, sizeof(uint32_t)*1024, &ctx->page_table_dma[index_pd], GFP_KERNEL | __GFP_ZERO);
			ctx->page_directory[index_pd] = 1 | (ctx->page_table_dma[index_pd] >> 8); 
		}


		int index_pt = address >> 12 & 0x3ff;
		ctx->page_tables[index_pd][index_pt] = 1 | (buffer->pages[i].addr >> 8) | (rdonly ? 0 : 2);
	}	

	return res;
}

long ioctl_unmap(struct uharddoom_context* ctx, uint32_t addr) {
	bool found = false;
	struct uharddoom_map* map;
	list_for_each_entry(map, &ctx->maps, lh) {
		if (map->virtual == addr) {
			found = true;
			break;
		}
	}
	if (!found) return -ENOENT;
	int i;
	for (i = 0; i < map->buffer->pages_nr; i++) {	
		
		uint32_t address = map->virtual + i * PAGE_SIZE;
		int index_pd = (address >> 22) & 0x3ff;
		if (!map->context->page_tables[index_pd]) {
			continue;
		}

		int index_pt = address >> 12 & 0x3ff;
		ctx->page_tables[index_pd][index_pt] = 0;
	}	
	uharddoom_iow(ctx->dev, UHARDDOOM_RESET, UHARDDOOM_RESET_TLB_USER);
	uharddoom_ior(ctx->dev, UHARDDOOM_STATUS);

	fput(map->file);
	list_del(&map->lh);
	kfree(map);
	return 0;
}
long ioctl_run(struct uharddoom_context* ctx, uint32_t addr, uint32_t size) {
	if ( addr % 4 != 0 || size % 4 != 0) return -EINVAL;
	
	struct uharddoom_device *dev = ctx->dev;
	long res = 0;
	unsigned long flags;
	struct uharddoom_job *ujob;
	/* Get a job  */
	spin_lock_irqsave(&dev->slock, flags);
	while (list_empty(&dev->jobs_free)) {
		spin_unlock_irqrestore(&dev->slock, flags);
		if (wait_event_interruptible(dev->free_wq, !list_empty(&dev->jobs_free)))
			return -ERESTARTSYS;
		spin_lock_irqsave(&dev->slock, flags);
	}
	ujob = list_entry(dev->jobs_free.next, struct uharddoom_job, lh);
	list_del(&ujob->lh);
	spin_unlock_irqrestore(&dev->slock, flags);
	ujob->size = size;
	ujob->ctx = ctx;
	ujob->addr = addr;
	ctx->pending_jobs++;
	/* Submit it.  */
	spin_lock_irqsave(&dev->slock, flags);
	if (list_empty(&dev->jobs_running)) {
		uharddoom_iow(dev, UHARDDOOM_JOB_PDP, (ctx->page_directory_dma) >> 12);
		uharddoom_iow(dev, UHARDDOOM_JOB_CMD_PTR, ujob->addr);
		uharddoom_iow(dev, UHARDDOOM_JOB_CMD_SIZE, ujob->size);
		uharddoom_iow(dev, UHARDDOOM_JOB_TRIGGER, 1);
	}
	list_add_tail(&ujob->lh, &dev->jobs_running);
	spin_unlock_irqrestore(&dev->slock, flags);
	return 0;	
}
long ioctl_wait(struct uharddoom_context* ctx, int32_t num_back) {

	struct uharddoom_device *dev = ctx->dev;
	unsigned long flags;
	spin_lock_irqsave(&dev->slock, flags);
	while (ctx->pending_jobs > num_back) {
		spin_unlock_irqrestore(&dev->slock, flags);
		if (wait_event_interruptible(ctx->wq, ctx->pending_jobs <= num_back))
			return -ERESTARTSYS;
		spin_lock_irqsave(&dev->slock, flags);
	}
	spin_unlock_irqrestore(&dev->slock, flags);
	return ctx->error ? -EIO : 0;

}
static long uharddoom_ioctl(struct file *filp, unsigned int cmd, 
                            unsigned long arg)
{
	struct uharddoom_context *ctx = filp->private_data;
        struct uharddoom_device *dev = ctx->dev;

	switch(cmd) {
		case UDOOMDEV_IOCTL_CREATE_BUFFER: 
		{
			struct udoomdev_ioctl_create_buffer ioctl_create_;
			if (copy_from_user(
                	  &ioctl_create_,
                	  (const void __user *)arg,
                	  sizeof(struct udoomdev_ioctl_create_buffer)
            		))
                		return -EFAULT;
			uint32_t size = ioctl_create_.size;
			return  ioctl_create(size, dev);
			break;
		}
		case UDOOMDEV_IOCTL_MAP_BUFFER:
                {
                        struct udoomdev_ioctl_map_buffer ioctl_map_;
                        if (copy_from_user(
                          &ioctl_map_,
                          (const void __user *)arg,
                          sizeof(struct udoomdev_ioctl_map_buffer)
                        ))
                                return -EFAULT;
                        unsigned int buf_fd = ioctl_map_.buf_fd;
			uint32_t map_rdonly = ioctl_map_.map_rdonly;
			mutex_lock(&ctx->mutex);
                        long res = ioctl_map(ctx, buf_fd, map_rdonly);
			mutex_unlock(&ctx->mutex);
			return res;
                }

		case UDOOMDEV_IOCTL_UNMAP_BUFFER:
                {
                        struct udoomdev_ioctl_unmap_buffer ioctl_unmap_;
                        if (copy_from_user(
                          &ioctl_unmap_,
                          (const void __user *)arg,
                          sizeof(struct udoomdev_ioctl_unmap_buffer)
                        ))
                                return -EFAULT;
                        uint32_t addr = ioctl_unmap_.addr;
			mutex_lock(&ctx->mutex);
                        long res = ioctl_unmap(ctx, addr);
			mutex_unlock(&ctx->mutex);
			return res;
                }

		case UDOOMDEV_IOCTL_RUN:
                {
                        struct udoomdev_ioctl_run ioctl_run_;
                        if (copy_from_user(
                          &ioctl_run_,
                          (const void __user *)arg,
                          sizeof(struct udoomdev_ioctl_run)
                        ))
                                return -EFAULT;
			uint32_t addr = ioctl_run_.addr;
                        uint32_t size = ioctl_run_.size;
                        return ioctl_run(ctx, addr, size);
                }

		case UDOOMDEV_IOCTL_WAIT:
                {
                        struct udoomdev_ioctl_wait ioctl_wait_;
                        if (copy_from_user(
                          &ioctl_wait_,
                          (const void __user *)arg,
                          sizeof(struct udoomdev_ioctl_wait)
                        ))
                                return -EFAULT;
                        uint32_t num_back = ioctl_wait_.num_back;
                        return ioctl_wait(ctx, num_back);
                        break;
                }
		default:
			return -ENOTTY;
	}
}

static const struct file_operations uharddoom_file_ops = {
	.owner = THIS_MODULE,
	.open = uharddoom_open,
	.release = uharddoom_release,
	.compat_ioctl = uharddoom_ioctl,
	.unlocked_ioctl = uharddoom_ioctl,
};

/* PCI driver.  */

static int uharddoom_probe(struct pci_dev *pdev,
	const struct pci_device_id *pci_id)
{
	int err, i;
	struct list_head *lh, *tmp;

	/* Allocate our structure.  */
	struct uharddoom_device *dev = kzalloc(sizeof *dev, GFP_KERNEL);
	if (!dev) {
		err = -ENOMEM;
		goto out_alloc;
	}
	pci_set_drvdata(pdev, dev);
	dev->pdev = pdev;

	/* Locks etc.  */
	spin_lock_init(&dev->slock);
	init_waitqueue_head(&dev->free_wq);
	init_waitqueue_head(&dev->idle_wq);
	INIT_LIST_HEAD(&dev->jobs_free);
	INIT_LIST_HEAD(&dev->jobs_running);

	/* Allocate a free index.  */
	mutex_lock(&uharddoom_devices_lock);
	for (i = 0; i < UHARDDOOM_MAX_DEVICES; i++)
		if (!uharddoom_devices[i])
			break;
	if (i == UHARDDOOM_MAX_DEVICES) {
		err = -ENOSPC; // XXX right?
		mutex_unlock(&uharddoom_devices_lock);
		goto out_slot;
	}
	uharddoom_devices[i] = dev;
	dev->idx = i;
	mutex_unlock(&uharddoom_devices_lock);

	/* Enable hardware resources.  */
	if ((err = pci_enable_device(pdev)))
		goto out_enable;

	if ((err = pci_set_dma_mask(pdev, DMA_BIT_MASK(32))))
		goto out_mask;
	if ((err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32))))
		goto out_mask;
	pci_set_master(pdev);

	if ((err = pci_request_regions(pdev, "uharddoom")))
		goto out_regions;

	/* Map the BAR.  */
	if (!(dev->bar = pci_iomap(pdev, 0, 0))) {
		err = -ENOMEM;
		goto out_bar;
	}

	/* Connect the IRQ line.  */
	if ((err = request_irq(pdev->irq, uharddoom_isr, IRQF_SHARED, "uharddoom", dev)))
		goto out_irq;

	/* Allocate some buffers.  TODO    */
	for (i = 0; i < UHARDDOOM_NUM_JOBS; i++) {
		struct uharddoom_job *job = kmalloc(sizeof *job, GFP_KERNEL);
		if (!job)
			goto out_cdev;
		job->ctx = 0;
		list_add(&job->lh, &dev->jobs_free);
	}
	uharddoom_iow(dev, UHARDDOOM_FE_CODE_ADDR, 0);
	
	for (i = 0; i < ARRAY_SIZE(udoomfw); i++) {
		uharddoom_iow(dev, UHARDDOOM_FE_CODE_WINDOW, udoomfw[i]);
	}
	uharddoom_iow(dev, UHARDDOOM_RESET, 0x7f7ffffe);
	uharddoom_iow(dev, UHARDDOOM_INTR, 0xff33);

	uharddoom_iow(dev, UHARDDOOM_INTR_ENABLE, 
			UHARDDOOM_INTR_JOB_DONE | UHARDDOOM_INTR_FE_ERROR |
			UHARDDOOM_INTR_CMD_ERROR | UHARDDOOM_INTR_PAGE_FAULT_BATCH |
			UHARDDOOM_INTR_PAGE_FAULT_CMD | UHARDDOOM_INTR_PAGE_FAULT_SRD |
			UHARDDOOM_INTR_PAGE_FAULT_SWR_DST | UHARDDOOM_INTR_PAGE_FAULT_COL_CMAP_B | 
			UHARDDOOM_INTR_PAGE_FAULT_COL_SRC | UHARDDOOM_INTR_PAGE_FAULT_SPAN_SRC |
			UHARDDOOM_INTR_PAGE_FAULT_SWR_TRANSMAP);
	uharddoom_iow(dev, UHARDDOOM_ENABLE, UHARDDOOM_ENABLE_ALL & ~UHARDDOOM_ENABLE_BATCH);	
	/* We're live.  Let's export the cdev.  */
	cdev_init(&dev->cdev, &uharddoom_file_ops);
	if ((err = cdev_add(&dev->cdev, uharddoom_devno + dev->idx, 1)))
		goto out_cdev;

	/* And register it in sysfs.  */
	dev->dev = device_create(&uharddoom_class,
			&dev->pdev->dev, uharddoom_devno + dev->idx, dev,
			"udoom%d", dev->idx);
	if (IS_ERR(dev->dev)) {
		printk(KERN_ERR "uharddoom: failed to register subdevice\n");
		/* too bad. */
		dev->dev = 0;
	}

	return 0;

out_cdev:
	uharddoom_iow(dev, UHARDDOOM_INTR_ENABLE, 0);
	uharddoom_iow(dev, UHARDDOOM_ENABLE, 0);
	uharddoom_ior(dev, UHARDDOOM_STATUS);
	list_for_each_safe(lh, tmp, &dev->jobs_free) {
		struct uharddoom_job *job = list_entry(lh, struct uharddoom_job, lh);
		kfree(job);
	}
	free_irq(pdev->irq, dev);
out_irq:
	pci_iounmap(pdev, dev->bar);
out_bar:
	pci_release_regions(pdev);
out_regions:
out_mask:
	pci_disable_device(pdev);
out_enable:
	mutex_lock(&uharddoom_devices_lock);
	uharddoom_devices[dev->idx] = 0;
	mutex_unlock(&uharddoom_devices_lock);
out_slot:
	kfree(dev);
out_alloc:
	return err;
}

static void uharddoom_remove(struct pci_dev *pdev)
{
	struct list_head *lh, *tmp;
	struct uharddoom_device *dev = pci_get_drvdata(pdev);
	if (dev->dev) {
		device_destroy(&uharddoom_class, uharddoom_devno + dev->idx);
	}
	cdev_del(&dev->cdev);
	uharddoom_iow(dev, UHARDDOOM_INTR_ENABLE, 0);
	uharddoom_iow(dev, UHARDDOOM_ENABLE, 0);
	uharddoom_ior(dev, UHARDDOOM_STATUS);
	list_for_each_safe(lh, tmp, &dev->jobs_free) {
		struct uharddoom_job *job = list_entry(lh, struct uharddoom_job, lh);
		kfree(job);
	}
	free_irq(pdev->irq, dev);
	pci_iounmap(pdev, dev->bar);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	mutex_lock(&uharddoom_devices_lock);
	uharddoom_devices[dev->idx] = 0;
	mutex_unlock(&uharddoom_devices_lock);
	kfree(dev);
}

static int uharddoom_suspend(struct pci_dev *pdev, pm_message_t state)
{
	unsigned long flags;
	struct uharddoom_device *dev = pci_get_drvdata(pdev);
	spin_lock_irqsave(&dev->slock, flags);
	while (!list_empty(&dev->jobs_running)) {
		spin_unlock_irqrestore(&dev->slock, flags);
		wait_event(dev->idle_wq, list_empty(&dev->jobs_running));
		spin_lock_irqsave(&dev->slock, flags);
	}
	spin_unlock_irqrestore(&dev->slock, flags);
	uharddoom_iow(dev, UHARDDOOM_INTR_ENABLE, 0);
	uharddoom_iow(dev, UHARDDOOM_ENABLE, 0);
	uharddoom_ior(dev, UHARDDOOM_STATUS);
	return 0;
}

static int uharddoom_resume(struct pci_dev *pdev)
{
	struct uharddoom_device *dev = pci_get_drvdata(pdev);
	int i;
	uharddoom_iow(dev, UHARDDOOM_FE_CODE_ADDR, 0);
	for ( i = 0; i < ARRAY_SIZE(udoomfw); i++) {
		uharddoom_iow(dev, UHARDDOOM_FE_CODE_WINDOW, udoomfw[i]);
	}
	uharddoom_iow(dev, UHARDDOOM_RESET, 0x7f7ffffe);
	uharddoom_iow(dev, UHARDDOOM_INTR, 0xff33);

	uharddoom_iow(dev, UHARDDOOM_INTR_ENABLE, 
			UHARDDOOM_INTR_JOB_DONE | UHARDDOOM_INTR_FE_ERROR |
			UHARDDOOM_INTR_CMD_ERROR | UHARDDOOM_INTR_PAGE_FAULT_BATCH |
			UHARDDOOM_INTR_PAGE_FAULT_CMD | UHARDDOOM_INTR_PAGE_FAULT_SRD |
			UHARDDOOM_INTR_PAGE_FAULT_SWR_DST | UHARDDOOM_INTR_PAGE_FAULT_COL_CMAP_B | 
			UHARDDOOM_INTR_PAGE_FAULT_COL_SRC | UHARDDOOM_INTR_PAGE_FAULT_SPAN_SRC |
			UHARDDOOM_INTR_PAGE_FAULT_SWR_TRANSMAP);
	uharddoom_iow(dev, UHARDDOOM_ENABLE, UHARDDOOM_ENABLE_ALL & ~UHARDDOOM_ENABLE_BATCH);	
	return 0;
}

static struct pci_device_id uharddoom_pciids[] = {
	{ PCI_DEVICE(UHARDDOOM_VENDOR_ID, UHARDDOOM_DEVICE_ID) },
	{ 0 }
};

static struct pci_driver uharddoom_pci_driver = {
	.name = "uharddoom",
	.id_table = uharddoom_pciids,
	.probe = uharddoom_probe,
	.remove = uharddoom_remove,
	.suspend = uharddoom_suspend,
	.resume = uharddoom_resume,
};
/* init & exit. */

static int uharddoom_init(void)
{
	int err;
	if ((err = alloc_chrdev_region(&uharddoom_devno, 0, UHARDDOOM_MAX_DEVICES, "uharddoom")))
		goto err_chrdev;
	if ((err = class_register(&uharddoom_class)))
		goto err_class;
	if ((err = pci_register_driver(&uharddoom_pci_driver)))
		goto err_pci;
	return 0;

err_pci:
	class_unregister(&uharddoom_class);
err_class:
	unregister_chrdev_region(uharddoom_devno, UHARDDOOM_MAX_DEVICES);
err_chrdev:
	return err;
}

static void uharddoom_exit(void)
{
	pci_unregister_driver(&uharddoom_pci_driver);
	class_unregister(&uharddoom_class);
	unregister_chrdev_region(uharddoom_devno, UHARDDOOM_MAX_DEVICES);
}

module_init(uharddoom_init);
module_exit(uharddoom_exit);
