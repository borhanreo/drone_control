import json
import sqlite3
conn = sqlite3.connect('drone.db',check_same_thread=False)


# def query_db(query, args=(), one=False):
#     cur = conn.cursor()
#     cur.execute(query, args)
#     r = [dict((cur.description[i][0], value) \
#               for i, value in enumerate(row)) for row in cur.fetchall()]
#     #cur.connection.close()
#     return (r[0] if r else None) if one else r


class DbInitializeClass:
    @staticmethod
    def dbInit():
        a=0
        c = conn.cursor()
        c.execute('''CREATE TABLE IF NOT EXISTS wp_status(id INTEGER PRIMARY KEY AUTOINCREMENT, uploaded int)''')
        conn.commit()
    @staticmethod
    def dbInitInsert():
        c = conn.cursor()
        rows = [(0, 1)]
        c.executemany('insert into wp_status values (?,?)', rows)
        conn.commit()
    @staticmethod
    def tbldbWaypoint():
        c = conn.cursor()
        c.execute('''CREATE TABLE IF NOT EXISTS wp(id INTEGER PRIMARY KEY, s_name TEXT,waypoint TEXT)''')
        conn.commit()
        #conn.close()

    @staticmethod
    def tbldbWaypointInsert(name, wp):
        task_1 = (name, wp)
        sql = ''' INSERT INTO wp(s_name,waypoint)VALUES(?,?) '''
        cur = conn.cursor()
        cur.execute(sql, task_1)
        conn.commit()
        print (cur.lastrowid)

    @staticmethod
    def select_all_wp():
        cur = conn.cursor()
        query = "SELECT id,s_name FROM wp"
        cur.execute(query)
        rows = cur.fetchall()
        for row in rows:
            print(row)
    @staticmethod
    def tbldbSchedule():
        c = conn.cursor()
        c.execute('''CREATE TABLE IF NOT EXISTS schedule(id INTEGER PRIMARY KEY, s_id INTEGER NOT NULL, drone_power_on INTEGER, flight_start INTEGER)''')
        conn.commit()
        # conn.close()

    @staticmethod
    def tbldbScheduleInsert(s_id,drone_power_on, flight_start):
        task_1 = (s_id, drone_power_on, flight_start)
        sql = ''' INSERT INTO schedule(s_id,drone_power_on,flight_start)VALUES(?,?,?) '''
        cur = conn.cursor()
        cur.execute(sql, task_1)
        conn.commit()
        print (cur.lastrowid)

    @staticmethod
    def select_all_tbldbSchedule():
        cur = conn.cursor()
        query = "SELECT * FROM schedule"
        cur.execute(query)

        rows = cur.fetchall()
        for row in rows:
            print(row)


    @staticmethod
    def get_waypoint_id_name_json_array(args=(), one=False):
        query = "SELECT id,s_name FROM wp"
        cur = conn.cursor()
        cur.execute(query)
        r = [dict((cur.description[i][0], value) \
                  for i, value in enumerate(row)) for row in cur.fetchall()]
        j2 = json.dumps((r[0] if r else None) if one else r)
        return j2

    @staticmethod
    def get_schedule_id_name_json_array(args=(), one=False):
        query = "SELECT id,s_id,drone_power_on,flight_start FROM schedule"
        cur = conn.cursor()
        cur.execute(query)
        r = [dict((cur.description[i][0], value) \
                  for i, value in enumerate(row)) for row in cur.fetchall()]
        j2 = json.dumps((r[0] if r else None) if one else r)

        return j2
    @staticmethod
    def fetch_all_row():
        cur = conn.cursor()
        cur.execute("SELECT id,s_id,drone_power_on,flight_start FROM schedule")
        rows = cur.fetchall()
        i = 0
        rtn = []
        for row in rows:
            rtn.append(row[3])
        return rtn

    @staticmethod
    def fetch_wp_by_sid(sid):
        cur = conn.cursor()
        cur.execute("SELECT waypoint FROM wp WHERE id=" + sid)
        rows = cur.fetchall()
        i = 0
        rtn = []
        for row in rows:
            print (row)
            #rtn.append(row[0])
        return rtn
    @staticmethod
    def fetch_sid_row(flight_start):
        cur = conn.cursor()
        cur.execute("SELECT s_id FROM schedule WHERE flight_start="+flight_start)
        rows = cur.fetchall()
        i = 0
        rtn = []
        for row in rows:
            rtn.append(row[0])
        return rtn

    @staticmethod
    def fetch_all_row_schedule(args=(), one=False):
        query = "SELECT *,(SELECT s_name FROM wp WHERE wp.id=schedule.s_id) AS sname  FROM schedule"
        cur = conn.cursor()
        cur.execute(query)
        r = [dict((cur.description[i][0], value) \
                  for i, value in enumerate(row)) for row in cur.fetchall()]
        j2 = json.dumps((r[0] if r else None) if one else r)
        return j2
    @staticmethod
    def get_schedule_counter():
        query = "SELECT count(*) FROM schedule"
        cur = conn.cursor()
        cur.execute(query)
        #dataCopy = sql.sqlExec("select count(*) from user")
        values = cur.fetchone()
        #print values[0]
        return values[0]
    @staticmethod
    def json_list(list):
        lst = []
        for pn in list:
            d = {}
            d['mpn'] = pn
            lst.append(d)
        return json.dumps(lst)

    #print json_list(part_nums)
    @staticmethod
    def delete_all_wp():
        cur = conn.cursor()
        cur.execute("DELETE FROM wp")
        conn.commit()
    @staticmethod
    def delete_wp_by_id(id):
        cur = conn.cursor()
        cur.execute("DELETE FROM wp WHERE id="+id)
        conn.commit()
        DbInitializeClass.delete_schedule_by_sid(id)
    @staticmethod
    def delete_schedule_by_id(id):
        cur = conn.cursor()
        cur.execute("DELETE FROM schedule WHERE id=" + id)
        conn.commit()
    @staticmethod
    def delete_schedule_by_sid(id):
        cur = conn.cursor()
        cur.execute("DELETE FROM schedule WHERE s_id=" + id)
        conn.commit()
    @staticmethod
    def delete_all_schedule():
        cur = conn.cursor()
        cur.execute("DELETE FROM schedule")
        conn.commit()
    @staticmethod
    def update_wp_status_false():
        c = conn.cursor()
        c.execute("UPDATE wp_status set uploaded=0 WHERE id=0")
        conn.commit()
        #conn.close()

    @staticmethod
    def update_wp_status_true():
        c = conn.cursor()
        c.execute("UPDATE wp_status set uploaded=1 WHERE id=0")
        conn.commit()
        #conn.close()

    @staticmethod
    def get_data_wp_status():
        c = conn.cursor()
        c.execute("SELECT * FROM wp_status")
        rows= c.fetchall()
        for row in rows:
            print(row[1])
            return row[1]
        #conn.close()