Week 6 Tutorial Questions
=========================

Before we start developing code that uses threading, we will have a look at an implementations of threading that allow us to understand basic concepts, we show how to do this on functions in [PREWORK](./PREWORK.md) and next in Class_Threading, before we go into more complex matters.

Class_Threading
-----------------------------------------

We have moved the Simple Threading code to a Class `Analysis`. There are few changes, mainly in how the threading starts which is now in the class itself. We moved from a `thread` to a `thread*` and also now need to indicate the `ClassName` associated to the function that is being subject to threading.

Consider:

* Would the main ever stop execution here? 
* What does the `sleep` do, what would happen if there is none?
* Check your CPU usage by using `top` , can you spot your process. What occurs if you reduce the sleep times.

There is nothing specific to edit in the code, just to observe how it runs, and we discuss in class.

Multi-Thread Data Access (DataBuffer)
-------------------------

We will run two threads in a class on functions  `removeValues` and `trimLength` that operate on vector `val_`

- `addValues`: keeps adding vector of values supplied to internal vector (to the end of it) and sorting it.
- `removeValues`:  removes numbers less than `minVal`and greater than `maxVal`
- `trimLength` keeps size of vector to max `trimSize` elements

There are two other functions that are not threaded. `getValues` returns the `val_` vector, while `addValues`  add a vector that has been passed as argument to the function to end of `val_` and  sort the vector via `bubbleSort` private function.

TASKS:
* Start the threads in the constructor, join them in destructor
* Given the vector is ordered, implement code to `removeValues` and make it thread safe (lock the mutex while working on vector and unlock when finished and before the sleep)
* Implement the `trimLength` function that removes elements from vector if it is grown over `trimSize_` length and make it thread safe (lock the mutex while working on vector and unlock when finished and before the sleep)

**Learning outcomes** (1) starting  multiple threads in object of class (2) we can not guarantee when threads run (3) providing a bit of time between locking/unlocking mutex in the same thread (be it a sleep or some heavy computing) will not `starve` the other thread

Multi-Thread Data Access (Teleop)
-------------------------

We have a very simplified class `Ackerman` which we want to operate continuously. The class is designed to send `UGV` commands of user specified`brake_,steer_,throttle`  continuously. And update them as the user presses key's on the keyboard.

This will allow us to develop a crude `teleoperation` concept. The user presses keys with functions are per below (terminating is `.` key)

```bash
    i o                i - increase throttle
 j  k  l               k - decrease throttle/increase brake
                       j - steer left
                       l - steer right
                       o - brake
                       . -exit
```

The main has the capability to read the keys and the task is to modify the `Ackerman` to drive it around.

TASK

* In Constructor of `Ackerman`create a `pfmsConnectorPtr_` and start a thread on the `drive` function. 
* In Destructor of `Ackerman` join thread (and force it to stop)
* In `drive`, lock the mutex, send `cmd_` via the `pipesPtr_`
* In `setCommand` store the values passed into `cmd`, don't forget the mutex.

You will need to run the simulator ``ros2 launch gazebo_tf audi_husky.launch.py` to be able to test your teleop.

**Learning outcomes** we show how to implement a thread-safe class, where functions of objects can be called and internally, it guarantees the variables are thread-safe. 
