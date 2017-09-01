import redis
import numpy as np

# note, currently stores database in dump.rdb in home directory

# simple example
# r = redis.StrictRedis(host='localhost', port=6379, db=0)
# print(r.set('foo', 'bar'))
# print(r.get('foo'))


# set seed for repeatability
np.random.seed(0)

# open the connection
r = redis.StrictRedis(host='172.24.68.162', port=6379, db=0)

# hierarchy:
# level 1 - task
# level 2 - user
# level 3 - keys for trajectory items (objectID, joints, etc)

num_tasks = 3
num_users = 4
num_objects = 5
num_data = 2

# set some keys for testing
print("\nPopulating db with test trajectories...")
for i in xrange(num_tasks):
    for j in xrange(num_users):
        for k in xrange(num_objects):
            r.set("task{}/user{}/obj{}".format(i+1, j+1, k+1), np.random.rand(num_data))

print("\nRetrieving all data for task 1...")
print("Note: we could use r.keys() for an easier implementation if we don't care about efficiency...\n")

task1_keys = []
scan_val = 0
while True:
    scan_val, keys = r.scan(scan_val, "task1/*")
    task1_keys.extend(keys)
    for key in keys:
        print("{} : {}".format(key, r.get(key)))
    if scan_val == 0:
        break

print("\nDeleting all data for task 1...\n")
for key in task1_keys:
    r.delete(key)

print("\nRetrieving all data for task 1 (should return nothing)...\n")
for key in task1_keys:
    print("{} : {}".format(key, r.get(key)))

# delete all keys that we added
scan_val = 0
while True:
    scan_val, keys = r.scan(scan_val, "task*")
    for key in keys:
        print("Deleteting key : {}".format(key))
        r.delete(key)
    if scan_val == 0:
        break


