#include "ccidd.h"
#include "../userspace/ccid.h"
#include "../userspace/stdlib.h"
#include "../userspace/file.h"
#include "../userspace/stdio.h"
#include "../userspace/block.h"
#include "../userspace/sys.h"
#include <string.h>
#include "sys_config.h"

typedef struct {
    HANDLE usb;
    HANDLE block;
    uint8_t data_ep, iface;
    uint8_t suspended;
} CCIDD;

static void ccidd_destroy(CCIDD* ccidd)
{
    block_destroy(ccidd->block);
    free(ccidd);
}

void ccidd_class_configured(USBD* usbd, USB_CONFIGURATION_DESCRIPTOR_TYPE* cfg)
{
    USB_INTERFACE_DESCRIPTOR_TYPE* iface;
    USB_ENDPOINT_DESCRIPTOR_TYPE* ep;
    uint8_t iface_num, data_ep;
    data_ep = iface_num = 0;

    for (iface = usb_get_first_interface(cfg); iface != NULL; iface = usb_get_next_interface(cfg, iface))
    {
        if (iface->bInterfaceClass == CCID_INTERFACE_CLASS)
        {
            //TODO: decode EPs (include interrupt)
            iface_num = iface->bInterfaceNumber;
            data_ep = 2;
            break;
/*            ep = (USB_ENDPOINT_DESCRIPTOR_TYPE*)usb_interface_get_first_descriptor(cfg, iface, USB_ENDPOINT_DESCRIPTOR_INDEX);
            if (ep != NULL)
            {
                in_ep = USB_EP_NUM(ep->bEndpointAddress);
                in_ep_size = ep->wMaxPacketSize;
                hid_iface = iface->bInterfaceNumber;
                break;
            }*/
        }
    }

    //No CCID descriptors in interface
    if (data_ep == 0)
        return;
    CCIDD* ccidd = (CCIDD*)malloc(sizeof(CCIDD));
    if (ccidd == NULL)
        return;
    ccidd->block = block_create(USB_CCID_BLOCK_SIZE);

    if (ccidd->block == INVALID_HANDLE)
    {
        ccidd_destroy(ccidd);
        return;
    }
    ccidd->usb = object_get(SYS_OBJ_USB);
    ccidd->iface = iface_num;
    ccidd->data_ep = data_ep;
    ccidd->suspended = false;

#if (USBD_DEBUG_CLASS_REQUESTS)
    printf("Found USB CCID device class, data: EP%d, iface: %d\n\r", ccidd->data_ep, ccidd->iface);
#endif //USBD_DEBUG_CLASS_REQUESTS

    //TODO:
    unsigned int size;
    size = 64;
    fopen_p(ccidd->usb, HAL_HANDLE(HAL_USB, USB_EP_IN | ccidd->data_ep), USB_EP_BULK, (void*)size);
    fopen_p(ccidd->usb, HAL_HANDLE(HAL_USB, ccidd->data_ep), USB_EP_BULK, (void*)size);
    usbd_register_interface(usbd, ccidd->iface, &__CCIDD_CLASS, ccidd);
    usbd_register_endpoint(usbd, ccidd->iface, ccidd->data_ep);

    //TODO: only if configured
    size = 8;
    fopen_p(ccidd->usb, HAL_HANDLE(HAL_USB, USB_EP_IN | 1), USB_EP_INTERRUPT, (void*)size);
    usbd_register_endpoint(usbd, ccidd->iface, ccidd->data_ep);

    //TODO:
    fread(ccidd->usb, HAL_HANDLE(HAL_USB, ccidd->data_ep), ccidd->block, 64);
}

void ccidd_class_reset(USBD* usbd, void* param)
{
    CCIDD* ccidd = (CCIDD*)param;

    fclose(ccidd->usb, HAL_HANDLE(HAL_USB, USB_EP_IN | ccidd->data_ep));
    fclose(ccidd->usb, HAL_HANDLE(HAL_USB, ccidd->data_ep));
    //TODO: only if configured
    fclose(ccidd->usb, HAL_HANDLE(HAL_USB, USB_EP_IN | ccidd->data_ep));
    usbd_unregister_endpoint(usbd, ccidd->iface, ccidd->data_ep);
    //TODO: only if configured
    usbd_unregister_endpoint(usbd, ccidd->iface, 1);
    usbd_unregister_interface(usbd, ccidd->iface, &__CCIDD_CLASS);

    ccidd_destroy(ccidd);
}

void ccidd_class_suspend(USBD* usbd, void* param)
{
    CCIDD* ccidd = (CCIDD*)param;
    fflush(ccidd->usb, HAL_HANDLE(HAL_USB, USB_EP_IN | ccidd->data_ep));
    fflush(ccidd->usb, HAL_HANDLE(HAL_USB, ccidd->data_ep));
    //TODO: only if configured
    fflush(ccidd->usb, HAL_HANDLE(HAL_USB, USB_EP_IN | ccidd->data_ep));
    ccidd->suspended = true;
}

void ccidd_class_resume(USBD* usbd, void* param)
{
    CCIDD* ccidd = (CCIDD*)param;
    ccidd->suspended = false;
    //TODO:
    fread(ccidd->usb, HAL_HANDLE(HAL_USB, ccidd->data_ep), ccidd->block, 64);
}

int ccidd_class_setup(USBD* usbd, void* param, SETUP* setup, HANDLE block)
{
    CCIDD* ccidd = (CCIDD*)param;
    unsigned int res = -1;
    switch (setup->bRequest)
    {
    default:
#if (USBD_DEBUG_CLASS_REQUESTS)
    printf("CCIDD SETUP request\n\r");
#endif //USBD_DEBUG_CLASS_REQUESTS
    }
    return res;
}

bool ccidd_class_request(USBD* usbd, void* param, IPC* ipc)
{
    CCIDD* ccidd = (CCIDD*)param;
    bool need_post = false;
    switch (ipc->cmd)
    {
    default:
#if (USBD_DEBUG_CLASS_REQUESTS)
    printf("CCIDD class request\n\r");
#endif //USBD_DEBUG_CLASS_REQUESTS
    }
    return need_post;
}

const USBD_CLASS __CCIDD_CLASS = {
    ccidd_class_configured,
    ccidd_class_reset,
    ccidd_class_suspend,
    ccidd_class_resume,
    ccidd_class_setup,
    ccidd_class_request,
};