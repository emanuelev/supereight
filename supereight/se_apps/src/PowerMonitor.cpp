/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include <stdio.h>
#include <perfstats.h>
#include "PowerMonitor.h"

PowerMonitor::PowerMonitor() {
	powerA7 = NULL;
	powerA15 = NULL;
	powerGPU = NULL;
	powerDRAM = NULL;
	sensingMethod = NONE;
	if (enableSensor(SENSOR_A7) == 0) {
		enableSensor(SENSOR_A15);
		enableSensor(SENSOR_GPU);
		enableSensor(SENSOR_DRAM);
		sensingMethod = ODROID;
		std::cerr << "Power sensors: ODROID" << std::endl;
	}
}
bool PowerMonitor::isActive() {
	if (sensingMethod == NONE)
		return (false);
	else
		return (true);
}
double PowerMonitor::start() {
	startTime = powerStats.get_time();
	return (startTime);
}
double PowerMonitor::sample() {
	double time = 0;
	if (sensingMethod == ODROID) {
		double a15 = getPower(SENSOR_A15);
		time = powerStats.sample("Sample_time",
				powerStats.get_time() - startTime, PerfStats::TIME);
		if (powerA7)
			powerStats.sample("Power_A7", getPower(SENSOR_A7),
					PerfStats::POWER);
		if (powerA15)
			powerStats.sample("Power_A15", a15, PerfStats::POWER);
		if (powerGPU)
			powerStats.sample("Power_GPU", getPower(SENSOR_GPU),
					PerfStats::POWER);
		if (powerDRAM)
			powerStats.sample("Power_DRAM", getPower(SENSOR_DRAM),
					PerfStats::POWER);
	}
	return (time);
}
PowerMonitor::~PowerMonitor() {
	if (powerA7)
		fclose(powerA7);
	if (powerA15)
		fclose(powerA15);
	if (powerGPU)
		fclose(powerGPU);
	if (powerDRAM)
		fclose(powerDRAM);
}

float PowerMonitor::getPower(Sensor sensor) {
	FILE *tmp;
	float power;
	if (sensingMethod == ODROID) {
		switch (sensor) {
		case SENSOR_A7:
			tmp = powerA7;
			break;
		case SENSOR_A15:
			tmp = powerA15;
			break;
		case SENSOR_GPU:
			tmp = powerGPU;
			break;
		case SENSOR_DRAM:
			tmp = powerDRAM;
			break;
		}
		if (tmp) {
			rewind(tmp);
			int res = fscanf(tmp, "%f\n", &power);
			return (power);
		}
	}

}

int PowerMonitor::enableSensor(Sensor sensor) {
	char enableFile[256];
	FILE *tmp;
	bool done = false;
	for (int dn = 1; dn < 5; dn++) {
		sprintf(enableFile, "/sys/bus/i2c/drivers/INA231/%d-00%d/enable", dn,
				sensor);

		if (tmp = fopen(enableFile, "a")) {
			fprintf(tmp, "1\n");
			fclose(tmp);

			sprintf(enableFile, "/sys/bus/i2c/drivers/INA231/%d-00%d/sensor_W",
					dn, sensor);
			if (tmp = fopen(enableFile, "r")) {
				switch (sensor) {
				case SENSOR_A7:
					powerA7 = tmp;
					break;
				case SENSOR_A15:
					powerA15 = tmp;
					break;
				case SENSOR_GPU:
					powerGPU = tmp;
					break;
				case SENSOR_DRAM:
					powerDRAM = tmp;
					break;
				}
				return (0);
			}
		}
	}
	return (1);
}

