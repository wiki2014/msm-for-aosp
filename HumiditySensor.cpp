/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *	  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>
#include <cutils/log.h>
#include <cutils/properties.h>

#include "sensors.h"
#include "HumiditySensor.h"

#define FETCH_FULL_EVENT_BEFORE_RETURN 	1

#define	EVENT_TYPE_PRESSURE			MSC_RAW
#define	EVENT_TYPE_TEMPERATURE		MSC_GESTURE
#define	EVENT_TYPE_HUMIDITY			MSC_SCAN

#define CONVERT_HUMIDITY		(0.01)
#define CONVERT_HUMIDITY_LPS25H	4096

#define IGNORE_EVENT_TIME		0

/*****************************************************************************/

HumiditySensor::HumiditySensor()
	: SensorBase(NULL, "Humidity"),
	  //mEnabled(0),
	  mInputReader(4),
	  mHasPendingEvent(false),
	  mEnabledTime(0)
{
	mPendingEvent.version = sizeof(sensors_event_t);
	mPendingEvent.sensor = SENSORS_RELATIVE_HUMIDITY_HANDLE;
	mPendingEvent.type = SENSOR_TYPE_RELATIVE_HUMIDITY;
	memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));

	if (data_fd) {
		strcpy(input_sysfs_path, "/sys/class/input/");
		strcat(input_sysfs_path, input_name);
//#ifdef TARGET_8610
		strcat(input_sysfs_path, "/device/");
//#else
//      strcat(input_sysfs_path, "/device/device/");
//#endif
		input_sysfs_path_len = strlen(input_sysfs_path);
		enable(0, 1);
	}
}

HumiditySensor::HumiditySensor(char *name)
	: SensorBase(NULL, "Humidity"),
	  //mEnabled(0),
	  mInputReader(4),
	  mHasPendingEvent(false),
	  mEnabledTime(0)
{
	mPendingEvent.version = sizeof(sensors_event_t);
	mPendingEvent.sensor = SENSORS_RELATIVE_HUMIDITY_HANDLE;
	mPendingEvent.type = SENSOR_TYPE_RELATIVE_HUMIDITY;
	memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));

	if (data_fd) {
		strlcpy(input_sysfs_path, SYSFS_CLASS, sizeof(input_sysfs_path));
		strlcat(input_sysfs_path, "/", sizeof(input_sysfs_path));
		strlcat(input_sysfs_path, name, sizeof(input_sysfs_path));
		strlcat(input_sysfs_path, "/", sizeof(input_sysfs_path));
		input_sysfs_path_len = strlen(input_sysfs_path);
		ALOGI("The Humidity sensor path is %s",input_sysfs_path);
		enable(0, 1);
	}
}

HumiditySensor::HumiditySensor(struct SensorContext *context)
	: SensorBase(NULL, NULL, context),
	  mInputReader(4),
	  mHasPendingEvent(false),
	  mEnabledTime(0)
{
	mPendingEvent.version = sizeof(sensors_event_t);
	mPendingEvent.sensor = context->sensor->handle;
	mPendingEvent.type = SENSOR_TYPE_RELATIVE_HUMIDITY;
	memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));
	data_fd = context->data_fd;
	strlcpy(input_sysfs_path, context->enable_path, sizeof(input_sysfs_path));
	input_sysfs_path_len = strlen(input_sysfs_path);
	mUseAbsTimeStamp = false;
	enable(0, 1);
}

HumiditySensor::~HumiditySensor() {
	if (mEnabled) {
		enable(0, 0);
	}
}

int HumiditySensor::setInitialState() {
	struct input_absinfo absinfo;
	float value;
	if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_HUMIDITY), &absinfo)) {
		value = absinfo.value;
		mPendingEvent.relative_humidity= value / CONVERT_HUMIDITY_LPS25H;
		mHasPendingEvent = true;
	}
	return 0;
}

int HumiditySensor::enable(int32_t, int en) {
	int flags = en ? 1 : 0;
	if (flags != mEnabled) {
		int fd;
		strlcpy(&input_sysfs_path[input_sysfs_path_len],
				SYSFS_ENABLE, SYSFS_MAXLEN);
		fd = open(input_sysfs_path, O_RDWR);
		if (fd >= 0) {
			char buf[2];
			int err;
			buf[1] = 0;
			if (flags) {
				buf[0] = '1';
				mEnabledTime = getTimestamp() + IGNORE_EVENT_TIME;
			} else {
				buf[0] = '0';
			}
			err = write(fd, buf, sizeof(buf));
			close(fd);
			mEnabled = flags;
			setInitialState();
			return 0;
		}
		return -1;
	}
	return 0;
}

bool HumiditySensor::hasPendingEvents() const {
	return mHasPendingEvent;
}

int HumiditySensor::setDelay(int32_t handle, int64_t delay_ns)
{
	int fd;
	int delay_ms = delay_ns / 1000000;
	strlcpy(&input_sysfs_path[input_sysfs_path_len],
			SYSFS_POLL_DELAY, SYSFS_MAXLEN);
	fd = open(input_sysfs_path, O_RDWR);
	if (fd >= 0) {
		char buf[80];
		sprintf(buf, "%d", delay_ms);
		write(fd, buf, strlen(buf)+1);
		close(fd);
		return 0;
	}
	return -1;
}

int HumiditySensor::readEvents(sensors_event_t* data, int count)
{
	if (count < 1)
		return -EINVAL;

	if (mHasPendingEvent) {
		mHasPendingEvent = false;
		mPendingEvent.timestamp = getTimestamp();
		*data = mPendingEvent;
		return mEnabled ? 1 : 0;
	}

	ssize_t n = mInputReader.fill(data_fd);
	if (n < 0)
		return n;

	int numEventReceived = 0;
	input_event const* event;

#if FETCH_FULL_EVENT_BEFORE_RETURN
again:
#endif
	while (count && mInputReader.readEvent(&event)) {
		int type = event->type;
		if (type == EV_MSC) {
			float value = event->value;
			if(event->code == EVENT_TYPE_HUMIDITY) {
				mPendingEvent.relative_humidity = value / 1000.0;//%RH
			}
		} else if (type == EV_SYN) {
			mPendingEvent.timestamp = timevalToNano(event->time);
			if (mEnabled) {
				if (mPendingEvent.timestamp >= mEnabledTime) {
					*data++ = mPendingEvent;
					numEventReceived++;
				}
				count--;
			}
		} else {
			ALOGE("HumiditySensor: unknown event (type=%d, code=%d)",
					type, event->code);
		}
		mInputReader.next();
	}

#if FETCH_FULL_EVENT_BEFORE_RETURN
	/* if we didn't read a complete event, see if we can fill and
	   try again instead of returning with nothing and redoing poll. */
	if (numEventReceived == 0 && mEnabled == 1) {
		n = mInputReader.fill(data_fd);
		if (n)
			goto again;
	}
#endif

	return numEventReceived;
}

