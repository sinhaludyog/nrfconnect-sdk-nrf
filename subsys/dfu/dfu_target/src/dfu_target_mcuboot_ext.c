/* Copyright (c) 2024-2026 Kimbal.io. All Rights Reserved. */
/*
 * This file is an extension of the dfu_target_mcuboot implementation that supports
 * targeting not only the secondary slot but also the primary slot.
 */

/*
 * Ensure 'strnlen' is available even with -std=c99. If
 * _POSIX_C_SOURCE was defined we will get a warning when referencing
 * 'strnlen'. If this proves to cause trouble we could easily
 * re-implement strnlen instead, perhaps with a different name, as it
 * is such a simple function.
 */
#if !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 200809L
#endif
#include <string.h>

#include <zephyr/kernel.h>
#include <pm_config.h>
#include <zephyr/logging/log.h>
#include <zephyr/dfu/mcuboot.h>
#include <dfu/dfu_target.h>
#include <dfu/dfu_target_stream.h>
#include <zephyr/devicetree.h>
#include <dfu_stream_flatten.h>

LOG_MODULE_REGISTER(dfu_target_mcuboot_ext, CONFIG_DFU_TARGET_LOG_LEVEL);

#define MCUBOOT_HEADER_MAGIC 0x96f3b83d

/* Begin secondary section */

#define IS_ALIGNED_32(POINTER) (((uintptr_t)(const void *)(POINTER)) % 4 == 0)

#define _MB_SEC_PAT(i, x) PM_MCUBOOT_SECONDARY_ ## i ## _ ## x

#define _MB_SEC_PAT_STRING(i, x) STRINGIFY(PM_MCUBOOT_SECONDARY_ ## i ## _ ## x)

#define _MB_SEC_PAT_DEV(i, x) COND_CODE_1(DT_NODE_EXISTS(PM_MCUBOOT_SECONDARY_##i##_##x), \
			      (DEVICE_DT_GET_OR_NULL(PM_MCUBOOT_SECONDARY_##i##_##x)), \
			      (DEVICE_DT_GET_OR_NULL(DT_NODELABEL(PM_MCUBOOT_SECONDARY_##i##_##x))))

#define _H_MB_SEC_LA(i) (PM_MCUBOOT_SECONDARY_## i ##_ADDRESS + \
			 PM_MCUBOOT_SECONDARY_## i ##_SIZE - 1)

#define _MB_SEC_LA(i, _) _H_MB_SEC_LA(i)

#define _STR_TARGET_NAME(i, _) STRINGIFY(MCUBOOT##i)

#ifdef PM_MCUBOOT_SECONDARY_2_ID
	#define TARGET_IMAGE_COUNT 3
#elif defined(PM_MCUBOOT_SECONDARY_1_ID)
	#define TARGET_IMAGE_COUNT 2
#else
	#define TARGET_IMAGE_COUNT 1
#endif

/* For the first image the Partition Manager doesn't use a 0 index.
 * Define helper macros with 0 to make the code more generic.
 */
#define PM_MCUBOOT_SECONDARY_0_SIZE	PM_MCUBOOT_SECONDARY_SIZE
#define PM_MCUBOOT_SECONDARY_0_ADDRESS	PM_MCUBOOT_SECONDARY_ADDRESS
#define PM_MCUBOOT_SECONDARY_0_NAME	STRINGIFY(PM_MCUBOOT_SECONDARY_NAME)
#define PM_MCUBOOT_SECONDARY_0_DEV	PM_MCUBOOT_SECONDARY_DEV

static const size_t secondary_size[] = {
	LIST_DROP_EMPTY(LISTIFY(TARGET_IMAGE_COUNT, _MB_SEC_PAT, (,), SIZE))
};

static const off_t secondary_address[] = {
	LIST_DROP_EMPTY(LISTIFY(TARGET_IMAGE_COUNT, _MB_SEC_PAT, (,), ADDRESS))
};

static const struct device *const secondary_dev[] = {
	LIST_DROP_EMPTY(LISTIFY(TARGET_IMAGE_COUNT, _MB_SEC_PAT_DEV, (,), DEV))
};

static const char *const secondary_name[] = {
	LIST_DROP_EMPTY(LISTIFY(TARGET_IMAGE_COUNT, _MB_SEC_PAT_STRING, (,), NAME))
};

/* End secondary section */

/* Begin primary section */

#define _MB_PRI_PAT(i, x) PM_MCUBOOT_PRIMARY_ ## i ## _ ## x

#define _MB_PRI_PAT_STRING(i, x) STRINGIFY(PM_MCUBOOT_PRIMARY_ ## i ## _ ## x)

#define _MB_PRI_PAT_DEV(i, x) COND_CODE_1(DT_NODE_EXISTS(PM_MCUBOOT_PRIMARY_##i##_##x), \
				  (DEVICE_DT_GET_OR_NULL(PM_MCUBOOT_PRIMARY_##i##_##x)), \
				  (DEVICE_DT_GET_OR_NULL(DT_NODELABEL(PM_MCUBOOT_PRIMARY_##i##_##x))))

#define _H_MB_PRI_LA(i) (PM_MCUBOOT_PRIMARY_## i ##_ADDRESS + \
		PM_MCUBOOT_PRIMARY_## i ##_SIZE - 1)

#define _MB_PRI_LA(i, _) _H_MB_PRI_LA(i)

enum dfu_slot_type {
	DFU_SLOT_PRIMARY = 0,
	DFU_SLOT_SECONDARY = 1,
};

#define PM_MCUBOOT_PRIMARY_0_SIZE	  PM_MCUBOOT_PRIMARY_SIZE
#define PM_MCUBOOT_PRIMARY_0_ADDRESS  PM_MCUBOOT_PRIMARY_ADDRESS
#define PM_MCUBOOT_PRIMARY_0_NAME	  STRINGIFY(PM_MCUBOOT_PRIMARY_NAME)
#define PM_MCUBOOT_PRIMARY_0_DEV	  PM_MCUBOOT_PRIMARY_DEV

static const size_t primary_size[] = {
	LIST_DROP_EMPTY(LISTIFY(TARGET_IMAGE_COUNT, _MB_PRI_PAT, (,), SIZE))
};

static const off_t primary_address[] = {
	LIST_DROP_EMPTY(LISTIFY(TARGET_IMAGE_COUNT, _MB_PRI_PAT, (,), ADDRESS))
};

static const struct device *const primary_dev[] = {
	LIST_DROP_EMPTY(LISTIFY(TARGET_IMAGE_COUNT, _MB_PRI_PAT_DEV, (,), DEV))
};

static const char *const primary_name[] = {
	LIST_DROP_EMPTY(LISTIFY(TARGET_IMAGE_COUNT, _MB_PRI_PAT_STRING, (,), NAME))
};

/* End primary section */

static uint8_t *stream_buf;
static size_t stream_buf_len;
static size_t stream_buf_bytes;
static uint8_t curr_sec_img;

bool dfu_target_mcuboot_ext_identify(const void *const buf)
{
	/* MCUBoot headers start with a 4 byte magic word. */
	return (*((const uint32_t *)buf) == MCUBOOT_HEADER_MAGIC);
}

int dfu_target_mcuboot_ext_set_buf(uint8_t *buf, size_t len)
{
	if (buf == NULL) {
		return -EINVAL;
	}

	if (!IS_ALIGNED_32(buf)) {
		return -EINVAL;
	}

	stream_buf = buf;
	stream_buf_len = len;

	return 0;
}

int dfu_target_mcuboot_ext_init(size_t file_size, int img_num, dfu_target_callback_t cb)
{
	ARG_UNUSED(cb);
	const struct device *flash_dev;
	int err;
	const char *name;
	size_t size;
	off_t address;

	stream_buf_bytes = 0;

	if (stream_buf == NULL) {
		LOG_ERR("Missing stream_buf, call '..set_buf' before '..init");
		return -ENODEV;
	}

	switch (img_num) {
	case DFU_SLOT_PRIMARY:
		flash_dev = primary_dev[0];
		name = primary_name[0];
		size = primary_size[0];
		address = primary_address[0];
		break;

	case DFU_SLOT_SECONDARY:
		flash_dev = secondary_dev[0];
		name = secondary_name[0];
		size = secondary_size[0];
		address = secondary_address[0];
		break;

	default:
		LOG_ERR("Invalid img_num %d range [0, 1]", img_num);
		return -EINVAL;
	}

	if (file_size > size) {
		LOG_ERR("Requested file too big to fit in flash %zu > 0x%x",
			file_size, size);
		return -EFBIG;
	}

	if (!device_is_ready(flash_dev)) {
		LOG_ERR("Failed to get device for area '%s'",
			name);
		return -EFAULT;
	}

	err = dfu_target_stream_init(&(struct dfu_target_stream_init){
		.id = name,
		.fdev = flash_dev,
		.buf = stream_buf,
		.len = stream_buf_len,
		.offset = address,
		.size = size,
		.cb = NULL });
	if (err < 0) {
		LOG_ERR("dfu_target_stream_init failed %d", err);
		return err;
	}

	curr_sec_img = img_num;
	return 0;
}

int dfu_target_mcuboot_ext_offset_get(size_t *out)
{
	int err = 0;

	err = dfu_target_stream_offset_get(out);
	if (err == 0) {
		*out += stream_buf_bytes;
	}

	return err;
}

int dfu_target_mcuboot_ext_write(const void *const buf, size_t len)
{
	stream_buf_bytes = (stream_buf_bytes + len) % stream_buf_len;

	return dfu_target_stream_write(buf, len);
}

int dfu_target_mcuboot_ext_done(bool successful)
{
	int err = 0;

	err = dfu_target_stream_done(successful);
	if (err != 0) {
		LOG_ERR("dfu_target_stream_done error %d", err);
		return err;
	}

	if (successful) {
		stream_buf_bytes = 0;
	} else {
		LOG_INF("MCUBoot image upgrade aborted.");
	}

	return err;
}

static int dfu_target_mcuboot_ext_schedule_one_img(int img_num)
{
	int err = 0;

	err = boot_request_upgrade_multi(img_num, BOOT_UPGRADE_TEST);
	if (err != 0) {
		LOG_ERR("boot_request_upgrade for image-%d error %d",
			img_num, err);
	} else {
		LOG_INF("MCUBoot image-%d upgrade scheduled. "
			"Reset device to apply", img_num);
	}

	return err;
}

int dfu_target_mcuboot_ext_schedule_update(int img_num)
{
	int err = 0;

	if (img_num == -1) {
		for (int i = 0; (i < TARGET_IMAGE_COUNT) && (err == 0); i++) {
			err = dfu_target_mcuboot_ext_schedule_one_img(i);
		}
	} else if ((img_num >= 0) && (img_num < TARGET_IMAGE_COUNT)) {
		err = dfu_target_mcuboot_ext_schedule_one_img(img_num);
	} else {
		err = -ENOENT;
	}

	return err;
}

int dfu_target_mcuboot_ext_reset(void)
{
	stream_buf_bytes = 0;
	return dfu_target_stream_reset();
}
