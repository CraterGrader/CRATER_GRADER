// firmware_update.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
// Include this header file to get access to VectorNav sensors.
#include "vn/sensors.h"

// We need this file for our sleep function.
#include "vn/thread.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

void one_call_method_for_the_VN300()
{
	// This example walks through using the VectorNav C++ Library to do a firmware update on the different processors of the VN300 using the firmwareUpdate method

	// First determine which COM port your sensor is attached to and update the constant below. Also, if you have changed your sensor from the factory
	// default baudrate of 115200, you will need to update the baudrate constant below as well.
	const string SensorPort = "COM4";                             // Windows format for physical and virtual (USB) serial port.
	// const string SensorPort = "/dev/ttyS1";                    // Linux format for physical serial port.
	// const string SensorPort = "/dev/ttyUSB0";                  // Linux format for virtual (USB) serial port.
	// const string SensorPort = "/dev/tty.usbserial-FTXXXXXX";   // Mac OS X format for virtual (USB) serial port.
	// const string SensorPort = "/dev/ttyS0";                    // CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1.
	const uint32_t SensorBaudrate = 115200;
	const uint32_t FirmwareUpdateBaudrate = 115200; // This can be different than the sensor baudrate, recommend 115200 or slower for reliability
	std::string VN300_NAVFirmwareUpdate("../../FirmwareUpdates/VN300_NAV_v0.5.0.0.vnx");
	std::string VN300_GPSFirmwareUpdate("../../FirmwareUpdates/VN300_GPS_v0.5.0.0.vnx");

	// Now let's create a VnSensor object and use it to connect to our sensor.
	cout << "Connecting to sensor" << endl;
	VnSensor vs;
	vs.connect(SensorPort, SensorBaudrate);

	// Let's query the sensor's model number.
	string mn = vs.readModelNumber();
	cout << "Model Number: " << mn << endl;

	// Let's query the sensor's firmware version.
	string fwv = vs.readFirmwareVersion();
	cout << "Firmware Version: " << fwv << endl;

	// Let's update the NAV firmware first.
	cout << "Updating NAV processor" << endl;
	vs.firmwareUpdate(FirmwareUpdateBaudrate, VN300_NAVFirmwareUpdate);

	// Let's switch to the target processor to update.
	cout << "Switching to the GPS processor to update" << endl;
	vs.switchProcessors(vs.VNPROCESSOR_GPS, mn, fwv);

	// Let's update the GPS firmware Second.
	cout << "Updating GPS processor" << endl;
	vs.firmwareUpdate(FirmwareUpdateBaudrate, VN300_GPSFirmwareUpdate);

	// Now disconnect from the sensor since we are finished.
	cout << "Disconnecting from sensor" << endl;
	vs.disconnect();

	// Depending on the sensor, a power cycle may be needed instead of a reset.
	cout << "Power Cycle the sensor" << endl;
}

void manage_each_step_method_for_the_VN200()
{
	// This example walks through using the VectorNav C++ Library to do a firmware update of the VN200 using the firmwareUpdate method

	// First determine which COM port your sensor is attached to and update the
	// constant below. Also, if you have changed your sensor from the factory
	// default baudrate of 115200, you will need to update the baudrate
	// constant below as well.
	const string SensorPort = "COM3";                             // Windows format for physical and virtual (USB) serial port.
	// const string SensorPort = "/dev/ttyS1";                    // Linux format for physical serial port.
	// const string SensorPort = "/dev/ttyUSB0";                  // Linux format for virtual (USB) serial port.
	// const string SensorPort = "/dev/tty.usbserial-FTXXXXXX";   // Mac OS X format for virtual (USB) serial port.
	// const string SensorPort = "/dev/ttyS0";                    // CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1.
	const uint32_t SensorBaudrate = 115200;
	const uint32_t FirmwareUpdateBaudrate = 115200; // This can be different than the sensor baudrate, recommend 115200 or slower for reliability
	std::string VN200FirmwareUpdate("../../FirmwareUpdates/VN200_NAV_v2.0.0.1.vnx");

	// Now let's create a VnSensor object and use it to connect to our sensor.
	cout << "Connecting to sensor" << endl;
	VnSensor vs;
	vs.connect(SensorPort, SensorBaudrate);

	// Let's query the sensor's model number.
	string mn = vs.readModelNumber();
	cout << "Model Number: " << mn << endl;

	// Let's query the sensor's firmware version.
	string fwv = vs.readFirmwareVersion();
	cout << "Firmware Version: " << fwv << endl;

	// Let's update the firmware on the target processor.
	cout << "Updating processor" << endl;
	std::string record = "";

	// Open firmware update file
	vs.openFirmwareUpdateFile(VN200FirmwareUpdate);

	// Enter bootloader mode
	vs.firmwareUpdateMode(true);

	// Give the sensor time to reboot into bootloader mode
	Thread::sleepSec(1);

	// Configure baudrate for firmware update
	uint32_t navBaudRate = vs.baudrate();
	vs.setFirmwareUpdateBaudRate(FirmwareUpdateBaudrate);

	// Calibrate the bootloader by letting it calculate the current baudrate.
	std::string bootloaderVersion = vs.calibrateBootloader();
	cout << bootloaderVersion << endl;

	// Send each record from the firmware update file one at a time
	do
	{
		record = vs.getNextFirwareUpdateRecord();
		if (!record.empty())
		{
			cout << ".";
			record.erase(0, 1); // Remove the ':' from the beginning of the line.
			vs.writeFirmwareUpdateRecord(record, true);
		}
	} while (!record.empty());
	cout << endl;

	// Close firmware update file
	vs.closeFirmwareUpdateFile();

	// Reset baudrate
	vs.setFirmwareUpdateBaudRate(navBaudRate);

	// Exit bootloader mode. Sleep for 10 seconds
	Thread::sleepSec(10);

	// Do a reset
	vs.reset();

	// Wait 2 seconds after a reset
	Thread::sleepSec(2);

	// Now disconnect from the sensor since we are finished.
	cout << "Disconnecting from sensor" << endl;
	vs.disconnect();

	// Depending on the sensor, a power cycle may be needed instead of a reset.
	cout << "Power Cycle the sensor" << endl;
}

void manage_each_step_method_for_the_VN310()
{
	const string SensorPort = "COM6";
	const uint32_t SensorBaudrate = 115200;
	const uint32_t FirmwareUpdateBaudrate = 115200; // This can be different than the sensor baudrate, recommend 115200 or slower for reliability
	std::string VN310_NAVFirmwareUpdate("../../FirmwareUpdates/VN310_NAV_v0.8.1.0.vnx");
	std::string VN310_GPSFirmwareUpdate("../../FirmwareUpdates/VN310_GPS_v0.8.1.0.vnx");
	std::string VN310_IMUFirmwareUpdate("../../FirmwareUpdates/VN310_IMU_v2.5.1.0.vnx");

	bool updateNAV = true;
	bool updateGPS = true;
	bool updateIMU = true;

	// Now let's create a VnSensor object and use it to connect to our sensor.
	cout << "Connecting to sensor" << endl;
	VnSensor vs;
	vs.connect(SensorPort, SensorBaudrate);

	// Let's query the sensor's model number.
	string mn = vs.readModelNumber();
	cout << "Model Number: " << mn << endl;

	// Let's query the sensor's firmware version.
	string fwv = vs.readFirmwareVersion();
	cout << "Firmware Version: " << fwv << endl;

	// Let's update the firmware on the target processor.
	cout << "Updating processor" << endl;
	std::string record = "";

	// ******************** Update NAV ******************** 
	if (updateNAV)
	{
		cout << "********************************************************************************" << endl;
		cout << "Updating the NAV firmware on a VN310" << endl;
		cout << "********************************************************************************" << endl;

		// Open firmware update file
		vs.openFirmwareUpdateFile(VN310_NAVFirmwareUpdate);

		// Enter bootloader mode
		vs.firmwareUpdateMode(true);

		// Give the sensor time to reboot into bootloader mode
		Thread::sleepSec(1);

		// Configure baudrate for firmware update
		uint32_t navBaudRate = vs.baudrate();
		vs.setFirmwareUpdateBaudRate(FirmwareUpdateBaudrate);

		// Calibrate the bootloader by letting it calculate the current baudrate.
		std::string bootloaderVersion = vs.calibrateBootloader();
		cout << bootloaderVersion << endl;

		// Send each record from the firmware update file one at a time
		do
		{
			record = vs.getNextFirwareUpdateRecord();
			if (!record.empty())
			{
				cout << ".";
				record.erase(0, 1); // Remove the ':' from the beginning of the line.
				vs.writeFirmwareUpdateRecord(record, true);
			}
		} while (!record.empty());

		// Close firmware update file
		vs.closeFirmwareUpdateFile();

		// Reset baudrate
		vs.setFirmwareUpdateBaudRate(navBaudRate);

		// Exit bootloader mode. Sleep for 10 seconds
		Thread::sleepSec(10);

		// Do a reset
		vs.reset();

		// Wait 2 seconds after a reset
		Thread::sleepSec(2);

		// Let's switch back to the main processor
		cout << "Switching to the NAV processor" << endl;
		vs.switchProcessors(vs.VNPROCESSOR_NAV, mn, fwv);
	}

	// ******************** Update GPS ******************** 
	if (updateGPS)
	{
		cout << "********************************************************************************" << endl;
		cout << "Updating the GPS firmware on a VN310" << endl;
		cout << "********************************************************************************" << endl;

		// Let's switch to the target processor to update.
		cout << "Switching to the GPS processor to update" << endl;
		vs.switchProcessors(vs.VNPROCESSOR_GPS, mn, fwv);

		// Open firmware update file
		vs.openFirmwareUpdateFile(VN310_GPSFirmwareUpdate);

		// Enter bootloader mode
		vs.firmwareUpdateMode(true);

		// Give the sensor time to reboot into bootloader mode
		Thread::sleepSec(1);

		// Configure baudrate for firmware update
		uint32_t navBaudRate = vs.baudrate();
		vs.setFirmwareUpdateBaudRate(FirmwareUpdateBaudrate);

		// Calibrate the bootloader by letting it calculate the current baudrate.
		std::string bootloaderVersion = vs.calibrateBootloader();
		cout << bootloaderVersion << endl;

		// Send each record from the firmware update file one at a time
		do
		{
			record = vs.getNextFirwareUpdateRecord();
			if (!record.empty())
			{
				cout << ".";
				record.erase(0, 1); // Remove the ':' from the beginning of the line.
				vs.writeFirmwareUpdateRecord(record, true);
			}
		} while (!record.empty());

		// Close firmware update file
		vs.closeFirmwareUpdateFile();

		// Reset baudrate
		vs.setFirmwareUpdateBaudRate(navBaudRate);

		// Exit bootloader mode. Sleep for 10 seconds
		Thread::sleepSec(10);

		// Do a reset
		vs.reset();

		// Wait 2 seconds after a reset
		Thread::sleepSec(2);

		// Let's switch back to the main processor
		cout << "Switching to the NAV processor" << endl;
		vs.switchProcessors(vs.VNPROCESSOR_NAV, mn, fwv);

	}

	// ******************** Update IMU ******************** 
	if (updateIMU)
	{
		cout << "********************************************************************************" << endl;
		cout << "Updating the IMU firmware on a VN310" << endl;
		cout << "********************************************************************************" << endl;

		// Let's switch to the target processor to update.
		cout << "Switching to the IMU processor to update" << endl;
		vs.switchProcessors(vs.VNPROCESSOR_IMU, mn, fwv);

		// Open firmware update file
		vs.openFirmwareUpdateFile(VN310_IMUFirmwareUpdate);

		// Enter bootloader mode
		vs.firmwareUpdateMode(true);

		// Give the sensor time to reboot into bootloader mode
		Thread::sleepSec(1);

		// Configure baudrate for firmware update
		uint32_t navBaudRate = vs.baudrate();
		vs.setFirmwareUpdateBaudRate(FirmwareUpdateBaudrate);

		// Calibrate the bootloader by letting it calculate the current baudrate.
		std::string bootloaderVersion = vs.calibrateBootloader();
		cout << bootloaderVersion << endl;

		// Send each record from the firmware update file one at a time
		do
		{
			record = vs.getNextFirwareUpdateRecord();
			if (!record.empty())
			{
				cout << ".";
				record.erase(0, 1); // Remove the ':' from the beginning of the line.
				vs.writeFirmwareUpdateRecord(record, true);
			}
		} while (!record.empty());

		// Close firmware update file
		vs.closeFirmwareUpdateFile();

		// Reset baudrate
		vs.setFirmwareUpdateBaudRate(navBaudRate);

		// Exit bootloader mode. Sleep for 10 seconds
		Thread::sleepSec(10);

		// Do a reset
		vs.reset();

		// Wait 2 seconds after a reset
		Thread::sleepSec(2);

		// Let's switch back to the main processor
		cout << "Switching to the NAV processor" << endl;
		vs.switchProcessors(vs.VNPROCESSOR_NAV, mn, fwv);
	}

	// Now disconnect from the sensor since we are finished.
	cout << "Disconnecting from sensor" << endl;
	vs.disconnect();
}


int main()
{
	cout << "Updating the firmware on a VN200" << endl;
	manage_each_step_method_for_the_VN200();

	cout << "Updating the firmware on a VN300" << endl;
	one_call_method_for_the_VN300();

	cout << "Updating the firmware on a VN310" << endl;
	manage_each_step_method_for_the_VN310();

	return 0;
}
