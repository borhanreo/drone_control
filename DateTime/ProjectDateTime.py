import datetime
class ProjectDateTimeClass:
    @staticmethod
    def _raspberry_pi_set_time(time_tuple):
        import ctypes
        import ctypes.util
        import time

        # /usr/include/linux/time.h:
        #
        # define CLOCK_REALTIME                     0
        CLOCK_REALTIME = 0

        # /usr/include/time.h
        #
        # struct timespec
        #  {
        #    __time_t tv_sec;            /* Seconds.  */
        #    long int tv_nsec;           /* Nanoseconds.  */
        #  };
        class timespec(ctypes.Structure):
            _fields_ = [("tv_sec", ctypes.c_long),
                        ("tv_nsec", ctypes.c_long)]

        librt = ctypes.CDLL(ctypes.util.find_library("rt"))

        ts = timespec()
        ts.tv_sec = int(time.mktime(datetime.datetime(*time_tuple[:6]).timetuple()))
        ts.tv_nsec = time_tuple[6] * 1000000  # Millisecond to nanosecond

        # http://linux.die.net/man/3/clock_settime
        librt.clock_settime(CLOCK_REALTIME, ctypes.byref(ts))
    @staticmethod
    def convert_date_time(timestamp):
        import datetime
        date = datetime.datetime.fromtimestamp(timestamp)
        time_tuple = (date.year,  # Year
                      date.month,  # Month
                      date.day,  # Day
                      date.hour,  # Hour
                      date.minute,  # Minute
                      date.second,  # Second
                      0,)  # Millisecond
        ProjectDateTimeClass._raspberry_pi_set_time(time_tuple)
        print (date.day, "/", date.month, "/", date.year, "   ", date.hour, ":", date.minute, ":", date.second)
