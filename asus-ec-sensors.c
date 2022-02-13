// SPDX-License-Identifier: GPL-2.0+
/*
 * HWMON driver for ASUS motherboards that publish some sensor values
 * via the embedded controller registers.
 *
 * Copyright (C) 2021 Eugene Shalygin <eugene.shalygin@gmail.com>

 * EC provides:
 * - Chipset temperature
 * - CPU temperature
 * - Motherboard temperature
 * - T_Sensor temperature
 * - VRM temperature
 * - Water In temperature
 * - Water Out temperature
 * - CPU Optional fan RPM
 * - Chipset fan RPM
 * - VRM Heat Sink fan RPM
 * - Water Flow fan RPM
 * - CPU current
 * - CPU core voltage
 */

#include <linux/acpi.h>
#include <linux/bitops.h>
#include <linux/dev_printk.h>
#include <linux/dmi.h>
#include <linux/hwmon.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sort.h>
#include <linux/units.h>

#include <asm/unaligned.h>

#ifndef MILLI
#define MILLI	1000UL
#endif

static char *mutex_path_override;

/* Writing to this EC register switches EC bank */
#define ASUS_EC_BANK_REGISTER	0xff
#define SENSOR_LABEL_LEN	16

/*
 * Arbitrary set max. allowed bank number. Required for sorting banks and
 * currently is overkill with just 2 banks used at max, but for the sake
 * of alignment let's set it to a higher value.
 */
#define ASUS_EC_MAX_BANK	3

#define ACPI_LOCK_DELAY_MS	500

/* ACPI mutex for locking access to the EC for the firmware */
#define ASUS_HW_ACCESS_MUTEX_ASMX	"\\AMW0.ASMX"

/* Moniker for the ACPI global lock (':' is not allowed in ASL identifiers */
#define ACPI_GLOBAL_LOCK_PSEUDO_PATH	":GLOBAL_LOCK"

/* There are two variants of the vendor spelling */
#define VENDOR_ASUS_UPPER_CASE	"ASUSTeK COMPUTER INC."

typedef union {
	u32 value;
	struct {
		u8 index;
		u8 bank;
		u8 size;
		u8 dummy;
	} components;
} sensor_address;

#define MAKE_SENSOR_ADDRESS(size, bank, index) {                               \
		.value = (size << 16) + (bank << 8) + index                    \
	}

static u32 hwmon_attributes[hwmon_max] = {
	[hwmon_chip] = HWMON_C_REGISTER_TZ,
	[hwmon_temp] = HWMON_T_INPUT | HWMON_T_LABEL,
	[hwmon_in] = HWMON_I_INPUT | HWMON_I_LABEL,
	[hwmon_curr] = HWMON_C_INPUT | HWMON_C_LABEL,
	[hwmon_fan] = HWMON_F_INPUT | HWMON_F_LABEL,
};

struct ec_sensor_info {
	char label[SENSOR_LABEL_LEN];
	enum hwmon_sensor_types type;
	sensor_address addr;
};

#define EC_SENSOR(sensor_label, sensor_type, size, bank, index) {              \
		.label = sensor_label, .type = sensor_type,                    \
		.addr = MAKE_SENSOR_ADDRESS(size, bank, index),                \
	}

enum ec_sensors {
	/* chipset temperature [℃] */
	ec_sensor_temp_chipset,
	/* CPU temperature [℃] */
	ec_sensor_temp_cpu,
	/* motherboard temperature [℃] */
	ec_sensor_temp_mb,
	/* "T_Sensor" temperature sensor reading [℃] */
	ec_sensor_temp_t_sensor,
	/* VRM temperature [℃] */
	ec_sensor_temp_vrm,
	/* CPU Core voltage [mV] */
	ec_sensor_in_cpu_core,
	/* CPU_Opt fan [RPM] */
	ec_sensor_fan_cpu_opt,
	/* VRM heat sink fan [RPM] */
	ec_sensor_fan_vrm_hs,
	/* Chipset fan [RPM] */
	ec_sensor_fan_chipset,
	/* Water flow sensor reading [RPM] */
	ec_sensor_fan_water_flow,
	/* CPU current [A] */
	ec_sensor_curr_cpu,
	/* "Water_In" temperature sensor reading [℃] */
	ec_sensor_temp_water_in,
	/* "Water_Out" temperature sensor reading [℃] */
	ec_sensor_temp_water_out,
};

#define SENSOR_TEMP_CHIPSET BIT(ec_sensor_temp_chipset)
#define SENSOR_TEMP_CPU BIT(ec_sensor_temp_cpu)
#define SENSOR_TEMP_MB BIT(ec_sensor_temp_mb)
#define SENSOR_TEMP_T_SENSOR BIT(ec_sensor_temp_t_sensor)
#define SENSOR_TEMP_VRM BIT(ec_sensor_temp_vrm)
#define SENSOR_IN_CPU_CORE BIT(ec_sensor_in_cpu_core)
#define SENSOR_FAN_CPU_OPT BIT(ec_sensor_fan_cpu_opt)
#define SENSOR_FAN_VRM_HS BIT(ec_sensor_fan_vrm_hs)
#define SENSOR_FAN_CHIPSET BIT(ec_sensor_fan_chipset)
#define SENSOR_FAN_WATER_FLOW BIT(ec_sensor_fan_water_flow)
#define SENSOR_CURR_CPU BIT(ec_sensor_curr_cpu)
#define SENSOR_TEMP_WATER_IN BIT(ec_sensor_temp_water_in)
#define SENSOR_TEMP_WATER_OUT BIT(ec_sensor_temp_water_out)

/* All the known sensors for ASUS EC controllers */
static const struct ec_sensor_info known_ec_sensors[] = {
	[ec_sensor_temp_chipset] =
		EC_SENSOR("Chipset", hwmon_temp, 1, 0x00, 0x3a),
	[ec_sensor_temp_cpu] = EC_SENSOR("CPU", hwmon_temp, 1, 0x00, 0x3b),
	[ec_sensor_temp_mb] =
		EC_SENSOR("Motherboard", hwmon_temp, 1, 0x00, 0x3c),
	[ec_sensor_temp_t_sensor] =
		EC_SENSOR("T_Sensor", hwmon_temp, 1, 0x00, 0x3d),
	[ec_sensor_temp_vrm] = EC_SENSOR("VRM", hwmon_temp, 1, 0x00, 0x3e),
	[ec_sensor_in_cpu_core] =
		EC_SENSOR("CPU Core", hwmon_in, 2, 0x00, 0xa2),
	[ec_sensor_fan_cpu_opt] =
		EC_SENSOR("CPU_Opt", hwmon_fan, 2, 0x00, 0xb0),
	[ec_sensor_fan_vrm_hs] = EC_SENSOR("VRM HS", hwmon_fan, 2, 0x00, 0xb2),
	[ec_sensor_fan_chipset] =
		EC_SENSOR("Chipset", hwmon_fan, 2, 0x00, 0xb4),
	[ec_sensor_fan_water_flow] =
		EC_SENSOR("Water_Flow", hwmon_fan, 2, 0x00, 0xbc),
	[ec_sensor_curr_cpu] = EC_SENSOR("CPU", hwmon_curr, 1, 0x00, 0xf4),
	[ec_sensor_temp_water_in] =
		EC_SENSOR("Water_In", hwmon_temp, 1, 0x01, 0x00),
	[ec_sensor_temp_water_out] =
		EC_SENSOR("Water_Out", hwmon_temp, 1, 0x01, 0x01),
};

/* Shortcuts for common combinations */
#define SENSOR_SET_TEMP_CHIPSET_CPU_MB                                         \
	(SENSOR_TEMP_CHIPSET | SENSOR_TEMP_CPU | SENSOR_TEMP_MB)
#define SENSOR_SET_TEMP_WATER (SENSOR_TEMP_WATER_IN | SENSOR_TEMP_WATER_OUT)

struct ec_board_info {
	unsigned long sensors;
	const char *mutex_path;
};

/* PRIME X570-PRO */
static const struct ec_board_info __initconst board_prime_x570_pro = {
	.sensors = SENSOR_SET_TEMP_CHIPSET_CPU_MB | SENSOR_TEMP_VRM |
		SENSOR_TEMP_T_SENSOR | SENSOR_FAN_CHIPSET,
	.mutex_path = ASUS_HW_ACCESS_MUTEX_ASMX,
};

/* Pro WS X570-ACE */
static const struct ec_board_info __initconst board_pro_ws_x570_ace = {
	.sensors = SENSOR_SET_TEMP_CHIPSET_CPU_MB | SENSOR_TEMP_VRM |
		SENSOR_FAN_CHIPSET | SENSOR_CURR_CPU | SENSOR_IN_CPU_CORE,
	.mutex_path = ASUS_HW_ACCESS_MUTEX_ASMX,
};

/* ROG CROSSHAIR VIII DARK HERO */
static const struct ec_board_info __initconst board_r_c8dh = {
	.sensors = SENSOR_SET_TEMP_CHIPSET_CPU_MB | SENSOR_TEMP_T_SENSOR |
		SENSOR_TEMP_VRM | SENSOR_SET_TEMP_WATER |
		SENSOR_FAN_CPU_OPT | SENSOR_FAN_WATER_FLOW |
		SENSOR_CURR_CPU | SENSOR_IN_CPU_CORE,
	.mutex_path = ASUS_HW_ACCESS_MUTEX_ASMX,
};

/* ROG CROSSHAIR VIII FORMULA */
static const struct ec_board_info __initconst board_r_c8f = {
	.sensors = SENSOR_SET_TEMP_CHIPSET_CPU_MB | SENSOR_TEMP_T_SENSOR |
		SENSOR_TEMP_VRM | SENSOR_FAN_CPU_OPT | SENSOR_FAN_CHIPSET |
		SENSOR_CURR_CPU | SENSOR_IN_CPU_CORE,
	.mutex_path = ASUS_HW_ACCESS_MUTEX_ASMX,
};

/* ROG CROSSHAIR VIII HERO */
static const struct ec_board_info __initconst board_r_c8h = {
	.sensors = SENSOR_SET_TEMP_CHIPSET_CPU_MB | SENSOR_TEMP_T_SENSOR |
		SENSOR_TEMP_VRM | SENSOR_SET_TEMP_WATER |
		SENSOR_FAN_CPU_OPT | SENSOR_FAN_CHIPSET |
		SENSOR_FAN_WATER_FLOW | SENSOR_CURR_CPU | SENSOR_IN_CPU_CORE,
	.mutex_path = ASUS_HW_ACCESS_MUTEX_ASMX,
};

/* ROG CROSSHAIR VIII IMPACT */
static const struct ec_board_info __initconst board_r_c8i = {
	.sensors = SENSOR_SET_TEMP_CHIPSET_CPU_MB | SENSOR_TEMP_T_SENSOR |
		SENSOR_TEMP_VRM | SENSOR_FAN_CHIPSET |
		SENSOR_CURR_CPU | SENSOR_IN_CPU_CORE,
	.mutex_path = ASUS_HW_ACCESS_MUTEX_ASMX,
};

/* ROG STRIX B550-E GAMING */
static const struct ec_board_info __initconst board_rs_b550_e_gaming = {
	.sensors = SENSOR_SET_TEMP_CHIPSET_CPU_MB |
		SENSOR_TEMP_T_SENSOR |
		SENSOR_TEMP_VRM | SENSOR_FAN_CPU_OPT,
	.mutex_path = ASUS_HW_ACCESS_MUTEX_ASMX,
};

/* ROG STRIX B550-I GAMING */
static const struct ec_board_info __initconst board_rs_b550_i_gaming = {
	.sensors = SENSOR_SET_TEMP_CHIPSET_CPU_MB |
		SENSOR_TEMP_T_SENSOR |
		SENSOR_TEMP_VRM | SENSOR_FAN_VRM_HS |
		SENSOR_CURR_CPU | SENSOR_IN_CPU_CORE,
	.mutex_path = ASUS_HW_ACCESS_MUTEX_ASMX,
};

/* ROG STRIX X570-E GAMING */
static const struct ec_board_info __initconst board_rs_x570_e_gaming = {
	.sensors = SENSOR_SET_TEMP_CHIPSET_CPU_MB |
		SENSOR_TEMP_T_SENSOR |
		SENSOR_TEMP_VRM | SENSOR_FAN_CHIPSET |
		SENSOR_CURR_CPU | SENSOR_IN_CPU_CORE,
	.mutex_path = ASUS_HW_ACCESS_MUTEX_ASMX,
};

/* ROG STRIX X570-F GAMING */
static const struct ec_board_info __initconst board_rs_x570_f_gaming = {
	.sensors = SENSOR_SET_TEMP_CHIPSET_CPU_MB |
		SENSOR_TEMP_T_SENSOR | SENSOR_FAN_CHIPSET,
	.mutex_path = ASUS_HW_ACCESS_MUTEX_ASMX,
};

/* ROG STRIX X570-I GAMING */
static const struct ec_board_info __initconst board_rs_x570_i_gaming = {
	.sensors = SENSOR_TEMP_T_SENSOR | SENSOR_FAN_VRM_HS |
		SENSOR_FAN_CHIPSET | SENSOR_CURR_CPU | SENSOR_IN_CPU_CORE,
	.mutex_path = ASUS_HW_ACCESS_MUTEX_ASMX,
};

#define DMI_EXACT_MATCH_BOARD(vendor, name, sensors) {                         \
	.matches = {                                                           \
		DMI_EXACT_MATCH(DMI_BOARD_VENDOR, vendor),                     \
		DMI_EXACT_MATCH(DMI_BOARD_NAME, name),                         \
	},                                                                     \
	.driver_data = (void *)(sensors), \
}

static const struct dmi_system_id asus_ec_dmi_table[] __initconst = {
	DMI_EXACT_MATCH_BOARD(VENDOR_ASUS_UPPER_CASE,
		"PRIME X570-PRO", &board_prime_x570_pro),
	DMI_EXACT_MATCH_BOARD(VENDOR_ASUS_UPPER_CASE,
		"Pro WS X570-ACE", &board_pro_ws_x570_ace),
	DMI_EXACT_MATCH_BOARD(VENDOR_ASUS_UPPER_CASE,
		"ROG CROSSHAIR VIII DARK HERO", &board_r_c8dh),
	DMI_EXACT_MATCH_BOARD(VENDOR_ASUS_UPPER_CASE,
		"ROG CROSSHAIR VIII FORMULA", &board_r_c8f),
	DMI_EXACT_MATCH_BOARD(VENDOR_ASUS_UPPER_CASE,
		"ROG CROSSHAIR VIII HERO", &board_r_c8h),
	DMI_EXACT_MATCH_BOARD(VENDOR_ASUS_UPPER_CASE, /* identical to C8H */
		"ROG CROSSHAIR VIII HERO (WI-FI)", &board_r_c8h),
	DMI_EXACT_MATCH_BOARD(VENDOR_ASUS_UPPER_CASE,
		"ROG CROSSHAIR VIII IMPACT", &board_r_c8i),
	DMI_EXACT_MATCH_BOARD(VENDOR_ASUS_UPPER_CASE,
		"ROG STRIX B550-E GAMING", &board_rs_b550_e_gaming),
	DMI_EXACT_MATCH_BOARD(VENDOR_ASUS_UPPER_CASE,
		"ROG STRIX B550-I GAMING", &board_rs_b550_i_gaming),
	DMI_EXACT_MATCH_BOARD(VENDOR_ASUS_UPPER_CASE,
		"ROG STRIX X570-E GAMING", &board_rs_x570_e_gaming),
	DMI_EXACT_MATCH_BOARD(VENDOR_ASUS_UPPER_CASE,
		"ROG STRIX X570-F GAMING", &board_rs_x570_f_gaming),
	DMI_EXACT_MATCH_BOARD(VENDOR_ASUS_UPPER_CASE,
		"ROG STRIX X570-I GAMING", &board_rs_x570_i_gaming),
	{}
};

struct ec_sensor {
	unsigned int info_index;
	s32 cached_value;
};

struct lock_data {
	union {
		acpi_handle aml;
		u32 global_lock_handle;
		struct mutex regular;
	} mutex;
	int (*lock)(struct lock_data *data);
	int (*unlock)(struct lock_data *data);
};

/*
 * The next functions pairs implement options for locking access to the
 * state and the EC
 */
static int lock_via_acpi_mutex(struct lock_data *data)
{
	/*
	 * ASUS DSDT does not specify that access to the EC has to be guarded,
	 * but firmware does access it via ACPI
	 */
	return acpi_acquire_mutex(data->mutex.aml, NULL,
				  ACPI_LOCK_DELAY_MS);
}

static int unlock_acpi_mutex(struct lock_data *data)
{
	return acpi_release_mutex(data->mutex.aml, NULL);
}

static int lock_via_global_acpi_lock(struct lock_data *data)
{
	return acpi_acquire_global_lock(ACPI_LOCK_DELAY_MS,
					&data->mutex.global_lock_handle);
}

static int unlock_global_acpi_lock(struct lock_data *data)
{
	return acpi_release_global_lock(data->mutex.global_lock_handle);
}

static int lock_via_mutex(struct lock_data *data)
{
	return mutex_trylock(&data->mutex.regular) ? 0 : -EBUSY;
}

static int unlock_mutex(struct lock_data *data)
{
	mutex_unlock(&data->mutex.regular);
	return 0;
}

struct ec_sensors_data {
	struct ec_board_info board_info;
	struct ec_sensor *sensors;
	/* EC registers to read from */
	u16 *registers;
	u8 *read_buffer;
	/* sorted list of unique register banks */
	u8 banks[ASUS_EC_MAX_BANK + 1];
	/* in jiffies */
	unsigned long last_updated;
	struct lock_data lock_data;
	/* number of board EC sensors */
	u8 nr_sensors;
	/*
	 * number of EC registers to read
	 * (sensor might span more than 1 register)
	 */
	u8 nr_registers;
	/* number of unique register banks */
	u8 nr_banks;
};

static u8 register_bank(u16 reg)
{
	return reg >> 8;
}

static u8 register_index(u16 reg)
{
	return reg & 0x00ff;
}

static bool is_sensor_data_signed(const struct ec_sensor_info *si)
{
	/*
	 * guessed from WMI functions in DSDT code for boards
	 * of the X470 generation
	 */
	return si->type == hwmon_temp;
}

static const struct ec_sensor_info *
get_sensor_info(const struct ec_sensors_data *state, int index)
{
	return &known_ec_sensors[state->sensors[index].info_index];
}

static int sensor_count(const struct ec_board_info *board)
{
	return hweight_long(board->sensors);
}

static int find_ec_sensor_index(const struct ec_sensors_data *ec,
				enum hwmon_sensor_types type, int channel)
{
	unsigned int i;

	for (i = 0; i < sensor_count(&ec->board_info); i++) {
		if (get_sensor_info(ec, i)->type == type) {
			if (channel == 0)
				return i;
			channel--;
		}
	}
	return -ENOENT;
}

static int __init bank_compare(const void *a, const void *b)
{
	return *((const s8 *)a) - *((const s8 *)b);
}

static void __init setup_sensor_data(struct ec_sensors_data *ec)
{
	struct ec_sensor *s = ec->sensors;
	bool bank_found;
	int i, j;
	u8 bank;

	ec->nr_banks = 0;
	ec->nr_registers = 0;

	for_each_set_bit(i, &ec->board_info.sensors,
			  BITS_PER_TYPE(ec->board_info.sensors)) {
		s->info_index = i;
		s->cached_value = 0;
		ec->nr_registers +=
			known_ec_sensors[s->info_index].addr.components.size;
		bank_found = false;
		bank = known_ec_sensors[s->info_index].addr.components.bank;
		for (j = 0; j < ec->nr_banks; j++) {
			if (ec->banks[j] == bank) {
				bank_found = true;
				break;
			}
		}
		if (!bank_found) {
			ec->banks[ec->nr_banks++] = bank;
		}
		s++;
	}
	sort(ec->banks, ec->nr_banks, 1, bank_compare, NULL);
}

static void __init fill_ec_registers(struct ec_sensors_data *ec)
{
	const struct ec_sensor_info *si;
	unsigned int i, j, register_idx = 0;

	for (i = 0; i < sensor_count(&ec->board_info); ++i) {
		si = get_sensor_info(ec, i);
		for (j = 0; j < si->addr.components.size; ++j, ++register_idx) {
			ec->registers[register_idx] =
				(si->addr.components.bank << 8) +
				si->addr.components.index + j;
		}
	}
}

static int __init setup_lock_data(struct device *dev)
{
	const char *mutex_path;
	int status;
	struct ec_sensors_data *state = dev_get_drvdata(dev);

	mutex_path = mutex_path_override ?
		mutex_path_override : state->board_info.mutex_path;

	if (!mutex_path || !strlen(mutex_path)) {
		mutex_init(&state->lock_data.mutex.regular);
		state->lock_data.lock = lock_via_mutex;
		state->lock_data.unlock = unlock_mutex;
		return 0;
	} else if (strcmp(mutex_path, ACPI_GLOBAL_LOCK_PSEUDO_PATH) == 0) {
		state->lock_data.lock = lock_via_global_acpi_lock;
		state->lock_data.unlock = unlock_global_acpi_lock;
		return 0;
	} else {
		status = acpi_get_handle(NULL, (acpi_string)mutex_path,
					 &state->lock_data.mutex.aml);
		if (ACPI_FAILURE(status)) {
			dev_err(dev,
				"Could not get hardware access guard mutex"
				"'%s': error %d",
				mutex_path, status);
			return -ENOENT;
		}
		state->lock_data.lock = lock_via_acpi_mutex;
		state->lock_data.unlock = unlock_acpi_mutex;
		return 0;
	}
}

static int asus_ec_bank_switch(u8 bank, u8 *old)
{
	int status = 0;

	if (old) {
		status = ec_read(ASUS_EC_BANK_REGISTER, old);
	}
	if (status || (old && (*old == bank)))
		return status;
	return ec_write(ASUS_EC_BANK_REGISTER, bank);
}

static int asus_ec_block_read(const struct device *dev,
			      struct ec_sensors_data *ec)
{
	int ireg, ibank, status;
	u8 bank, reg_bank, prev_bank;

	bank = 0;
	status = asus_ec_bank_switch(bank, &prev_bank);
	if (status) {
		dev_warn(dev, "EC bank switch failed");
		return status;
	}

	if (prev_bank) {
		/* oops... somebody else is working with the EC too */
		dev_warn(dev,
			"Concurrent access to the ACPI EC detected.\n"
			"Race condition possible.");
	}

	/* read registers minimizing bank switches. */
	for (ibank = 0; ibank < ec->nr_banks; ibank++) {
		if (bank != ec->banks[ibank]) {
			bank = ec->banks[ibank];
			if (asus_ec_bank_switch(bank, NULL)) {
				dev_warn(dev, "EC bank switch to %d failed",
					 bank);
				break;
			}
		}
		for (ireg = 0; ireg < ec->nr_registers; ireg++) {
			reg_bank = register_bank(ec->registers[ireg]);
			if (reg_bank < bank) {
				continue;
			}
			ec_read(register_index(ec->registers[ireg]),
				ec->read_buffer + ireg);
		}
	}

	status = asus_ec_bank_switch(prev_bank, NULL);
	return status;
}

static inline s32 get_sensor_value(const struct ec_sensor_info *si, u8 *data)
{
	if (is_sensor_data_signed(si)) {
		switch (si->addr.components.size) {
		case 1:
			return (s8)*data;
		case 2:
			return (s16)get_unaligned_be16(data);
		case 4:
			return (s32)get_unaligned_be32(data);
		default:
			return 0;
		}
	} else {
		switch (si->addr.components.size) {
		case 1:
			return *data;
		case 2:
			return get_unaligned_be16(data);
		case 4:
			return get_unaligned_be32(data);
		default:
			return 0;
		}
	}
}

static void update_sensor_values(struct ec_sensors_data *ec, u8 *data)
{
	const struct ec_sensor_info *si;
	struct ec_sensor *s, *sensor_end;

	sensor_end = ec->sensors + sensor_count(&ec->board_info);
	for (s = ec->sensors; s != sensor_end; s++) {
		si = &known_ec_sensors[s->info_index];
		s->cached_value = get_sensor_value(si, data);
		data += si->addr.components.size;
	}
}

static int update_ec_sensors(const struct device *dev,
			     struct ec_sensors_data *ec)
{
	int status;

	if (ec->lock_data.lock(&ec->lock_data)) {
		dev_err(dev, "Failed to acquire mutex");
		return -EBUSY;
	}

	status = asus_ec_block_read(dev, ec);

	if (!status) {
		update_sensor_values(ec, ec->read_buffer);
	}

	if (ec->lock_data.unlock(&ec->lock_data)) {
		dev_err(dev, "Failed to release mutex");
	}

	return status;
}

static long scale_sensor_value(s32 value, int data_type)
{
	switch (data_type) {
	case hwmon_curr:
	case hwmon_temp:
		return value * MILLI;
	default:
		return value;
	}
}

static int get_cached_value_or_update(const struct device *dev,
				      int sensor_index,
				      struct ec_sensors_data *state, s32 *value)
{
	if (time_after(jiffies, state->last_updated + HZ)) {
		if (update_ec_sensors(dev, state)) {
			dev_err(dev, "update_ec_sensors() failure\n");
			return -EIO;
		}

		state->last_updated = jiffies;
	}

	*value = state->sensors[sensor_index].cached_value;
	return 0;
}

/*
 * Now follow the functions that implement the hwmon interface
 */

static int asus_ec_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			      u32 attr, int channel, long *val)
{
	int ret;
	s32 value = 0;

	struct ec_sensors_data *state = dev_get_drvdata(dev);
	int sidx = find_ec_sensor_index(state, type, channel);

	if (sidx < 0) {
		return sidx;
	}

	ret = get_cached_value_or_update(dev, sidx, state, &value);
	if (!ret) {
		*val = scale_sensor_value(value,
					  get_sensor_info(state, sidx)->type);
	}

	return ret;
}

static int asus_ec_hwmon_read_string(struct device *dev,
				     enum hwmon_sensor_types type, u32 attr,
				     int channel, const char **str)
{
	struct ec_sensors_data *state = dev_get_drvdata(dev);
	int sensor_index = find_ec_sensor_index(state, type, channel);
	*str = get_sensor_info(state, sensor_index)->label;

	return 0;
}

static umode_t asus_ec_hwmon_is_visible(const void *drvdata,
					enum hwmon_sensor_types type, u32 attr,
					int channel)
{
	const struct ec_sensors_data *state = drvdata;

	return find_ec_sensor_index(state, type, channel) >= 0 ? S_IRUGO : 0;
}

static int __init
asus_ec_hwmon_add_chan_info(struct hwmon_channel_info *asus_ec_hwmon_chan,
			     struct device *dev, int num,
			     enum hwmon_sensor_types type, u32 config)
{
	int i;
	u32 *cfg = devm_kcalloc(dev, num + 1, sizeof(*cfg), GFP_KERNEL);

	if (!cfg)
		return -ENOMEM;

	asus_ec_hwmon_chan->type = type;
	asus_ec_hwmon_chan->config = cfg;
	for (i = 0; i < num; i++, cfg++)
		*cfg = config;

	return 0;
}

static const struct hwmon_ops asus_ec_hwmon_ops = {
	.is_visible = asus_ec_hwmon_is_visible,
	.read = asus_ec_hwmon_read,
	.read_string = asus_ec_hwmon_read_string,
};

static struct hwmon_chip_info asus_ec_chip_info = {
	.ops = &asus_ec_hwmon_ops,
};

static const struct ec_board_info * __init
get_board_info(const struct device *dev)
{
	const struct dmi_system_id *dmi_entry;

	dmi_entry = dmi_first_match(asus_ec_dmi_table);
	if (!dmi_entry) {
		dev_info(dev, "Unsupported board");
		return NULL;
	}

	return dmi_entry->driver_data;
}

static int __init configure_sensor_setup(struct device *dev)
{
	struct ec_sensors_data *ec_data = dev_get_drvdata(dev);
	int nr_count[hwmon_max] = { 0 }, nr_types = 0;
	const struct ec_board_info *pboard_info;
	struct device *hwdev;
	struct hwmon_channel_info *asus_ec_hwmon_chan;
	const struct hwmon_channel_info **ptr_asus_ec_ci;
	const struct hwmon_chip_info *chip_info;
	const struct ec_sensor_info *si;
	enum hwmon_sensor_types type;
	unsigned int i;
	int status;

	pboard_info = get_board_info(dev);
	if (!pboard_info) {
		return -ENODEV;
	}
	ec_data->board_info = *pboard_info;
	status = setup_lock_data(dev);
	if (status) {
		dev_err(dev, "Failed to setup state/EC locking: %d", status);
		return status;
	}

	ec_data->sensors = devm_kcalloc(dev,
					 sensor_count(&ec_data->board_info),
					sizeof(struct ec_sensor), GFP_KERNEL);

	setup_sensor_data(ec_data);
	ec_data->registers = devm_kcalloc(dev, ec_data->nr_registers,
					  sizeof(u16), GFP_KERNEL);
	ec_data->read_buffer = devm_kcalloc(dev, ec_data->nr_registers,
					    sizeof(u8), GFP_KERNEL);

	if (!ec_data->registers || !ec_data->read_buffer) {
		return -ENOMEM;
	}

	fill_ec_registers(ec_data);

	for (i = 0; i < sensor_count(&ec_data->board_info); ++i) {
		si = get_sensor_info(ec_data, i);
		if (!nr_count[si->type])
			++nr_types;
		++nr_count[si->type];
	}

	if (nr_count[hwmon_temp])
		nr_count[hwmon_chip]++, nr_types++;

	asus_ec_hwmon_chan = devm_kcalloc(
		dev, nr_types, sizeof(*asus_ec_hwmon_chan), GFP_KERNEL);
	if (!asus_ec_hwmon_chan)
		return -ENOMEM;

	ptr_asus_ec_ci = devm_kcalloc(dev, nr_types + 1,
				       sizeof(*ptr_asus_ec_ci), GFP_KERNEL);
	if (!ptr_asus_ec_ci)
		return -ENOMEM;

	asus_ec_chip_info.info = ptr_asus_ec_ci;
	chip_info = &asus_ec_chip_info;

	for (type = 0; type < hwmon_max; ++type) {
		if (!nr_count[type])
			continue;

		asus_ec_hwmon_add_chan_info(asus_ec_hwmon_chan, dev,
					     nr_count[type], type,
					     hwmon_attributes[type]);
		*ptr_asus_ec_ci++ = asus_ec_hwmon_chan++;
	}

	dev_info(dev, "board has %d EC sensors that span %d registers",
		 sensor_count(&ec_data->board_info), ec_data->nr_registers);

	hwdev = devm_hwmon_device_register_with_info(dev, "asusec",
						     ec_data, chip_info, NULL);

	return PTR_ERR_OR_ZERO(hwdev);
}

static int __init asus_ec_probe(struct platform_device *pdev)
{
	struct ec_sensors_data *state;
	int status = 0;

	state = devm_kzalloc(&pdev->dev, sizeof(struct ec_sensors_data),
			     GFP_KERNEL);

	if (!state) {
		return -ENOMEM;
	}

	dev_set_drvdata(&pdev->dev, state);
	status = configure_sensor_setup(&pdev->dev);
	return status;
}

static const struct acpi_device_id acpi_ec_ids[] = {
	/* Embedded Controller Device */
	{ "PNP0C09", 0 },
	{}
};

static struct platform_driver asus_ec_sensors_platform_driver = {
	.driver = {
		.name	= "asus-ec-sensors",
		.acpi_match_table = acpi_ec_ids,
	},
};

MODULE_DEVICE_TABLE(dmi, asus_ec_dmi_table);
module_platform_driver_probe(asus_ec_sensors_platform_driver, asus_ec_probe);

module_param_named(mutex_path, mutex_path_override, charp, 0);
MODULE_PARM_DESC(mutex_path,
		 "Override ACPI mutex path used to guard access to hardware");

MODULE_AUTHOR("Eugene Shalygin <eugene.shalygin@gmail.com>");
MODULE_DESCRIPTION(
	"HWMON driver for sensors accessible via ACPI EC in ASUS motherboards");
MODULE_LICENSE("GPL");
