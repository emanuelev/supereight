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
	sensing_method = NONE;
	if (enableSensor(SENSOR_A7) == 0) {
		enableSensor(SENSOR_A15);
		enableSensor(SENSOR_GPU);
		enableSensor(SENSOR_DRAM);
		sensing_method = ODROID;
		std::cerr << "Power sensors: ODROID" << std::endl;
	}
}
bool PowerMonitor::isActive() {
	if (sensing_method == NONE)
		return (false);
	else
		return (true);
}
double PowerMonitor::start() {
	start_time = power_stats.get_time();
	return (start_time);
}
double PowerMonitor::sample() {
	double time = 0;
	if (sensing_method == ODROID) {
		double a15 = getPower(SENSOR_A15);
		time = power_stats.sample("Sample_time",
				power_stats.get_time() - start_time, PerfStats::TIME);
		if (powerA7)
			power_stats.sample("Power_A7", getPower(SENSOR_A7),
					PerfStats::POWER);
		if (powerA15)
			power_stats.sample("Power_A15", a15, PerfStats::POWER);
		if (powerGPU)
			power_stats.sample("Power_GPU", getPower(SENSOR_GPU),
					PerfStats::POWER);
		if (powerDRAM)
			power_stats.sample("Power_DRAM", getPower(SENSOR_DRAM),
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
	if (sensing_method == ODROID) {
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

