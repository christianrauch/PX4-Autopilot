#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(0, {
//		initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, 0),
//		initSPIDevice(DRV_IMU_DEVTYPE_ICM42605, 0),
	}),
};
