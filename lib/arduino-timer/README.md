# arduino-timer - library for delaying function calls

Simple *non-blocking* timer library for calling functions **in / at / every** specified units of time. Supports millis, micros, time rollover, and compile time configurable number of tasks.

### Use It

Include the library and create a *Timer* instance.
```cpp
#include <arduino-timer.h>

auto timer = timer_create_default();
```

Or using the *Timer* constructors for different task limits / time resolution
```cpp
Timer<10> timer; // 10 concurrent tasks, using millis as resolution
Timer<10, micros> timer; // 10 concurrent tasks, using micros as resolution
Timer<10, micros, int> timer; // 10 concurrent tasks, using micros as resolution, with handler argument of type int
```

Call *timer*.**tick()** in the loop function
```cpp
void loop() {
    timer.tick();
}
```

Make a function to call when the *Timer* expires
```cpp
bool function_to_call(void *argument /* optional argument given to in/at/every */) {
    return true; // to repeat the action - false to stop
}
```

Call *function\_to\_call* **in** *delay* units of time *(unit of time defaults to milliseconds)*.
```cpp
timer.in(delay, function_to_call);
timer.in(delay, function_to_call, argument); // or with an optional argument for function_to_call
```

Call *function\_to\_call* **at** a specific *time*.
```cpp
timer.at(time, function_to_call);
timer.at(time, function_to_call, argument); // with argument
```

Call *function\_to\_call* **every** *interval* units of time.
```cpp
timer.every(interval, function_to_call);
timer.every(interval, function_to_call, argument); // with argument
```

To **cancel** a *Task*
```cpp
auto task = timer.in(delay, function_to_call);
timer.cancel(task);
```

To **cancel** all *Task*s
```cpp
timer.cancel();
```

Check if a timer is **empty** - no active *Task*s
```cpp
if (timer.empty()) { /* no active tasks */ }
```

Get the number of active *Task*s
```cpp
auto active_tasks = timer.size();
```

Be fancy with **lambdas**
```cpp
timer.in(1000, [](void*) -> bool { return false; });
timer.in(1000, [](void *argument) -> bool { return argument; }, argument);
```

Getting the number of **ticks** until the next *Task*
```cpp
auto ticks = timer.ticks(); // usefull for sleeping until the next task
```
```cpp
void loop {
    auto ticks = timer.tick(); // returns the number of ticks
}
```

Avoiding **ticks** calculation inside of **tick**
```cpp
void loop {
    timer.tick<void>(); // avoids ticks() calculation
}
```

### API

```cpp
/* Constructors */
/* Create a timer object with default settings:
   millis resolution, TIMER_MAX_TASKS (=16) task slots, T = void *
*/
Timer<> timer_create_default(); // auto timer = timer_create_default();

/* Create a timer with max_tasks slots and time_func resolution */
Timer<size_t max_tasks = TIMER_MAX_TASKS, unsigned long (*time_func)(void) = millis, typename T = void *> timer;
Timer<> timer; // Equivalent to: auto timer = timer_create_default()
Timer<10> timer; // Timer with 10 task slots
Timer<10, micros> timer; // timer with 10 task slots and microsecond resolution
Timer<10, micros, int> timer; // timer with 10 task slots, microsecond resolution, and handler argument type int

/* Signature for handler functions - T = void * by default */
bool handler(T argument);

/* Timer Methods */
/* Ticks the timer forward, returns the ticks until next event, or 0 if none */
unsigned long tick(); // call this function in loop()

/* Calls handler with opaque as argument in delay units of time */
Timer<>::Task
in(unsigned long delay, handler_t handler, T opaque = T());

/* Calls handler with opaque as argument at time */
Timer<>::Task
at(unsigned long time, handler_t handler, T opaque = T());

/* Calls handler with opaque as argument every interval units of time */
Timer<>::Task
every(unsigned long interval, handler_t handler, T opaque = T());

/* Cancel a timer task */
void cancel(Timer<>::Task &task);
/* Cancel all tasks */
void cancel();

/* Returns the ticks until next event, or 0 if none */
unsigned long ticks();

/* Number of active tasks in the timer */
size_t size() const;

/* True if there are no active tasks */
bool empty() const;
```

### Installation

[Check out the instructions](https://www.arduino.cc/en/Guide/Libraries) from Arduino.

**OR** copy **src/arduino-timer.h** into your project folder *(you won't get managed updates this way)*.

### Examples

Found in the **examples/** folder.

The simplest example, blinking an LED every second *(from examples/blink)*:

```cpp
#include <arduino-timer.h>

auto timer = timer_create_default(); // create a timer with default settings

bool toggle_led(void *) {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // toggle the LED
  return true; // keep timer active? true
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // set LED pin to OUTPUT

  // call the toggle_led function every 1000 millis (1 second)
  timer.every(1000, toggle_led);
}

void loop() {
  timer.tick(); // tick the timer
}
```

### LICENSE

Check the LICENSE file - 3-Clause BSD License

### Notes

Currently only a software timer. Any blocking code delaying *timer*.**tick()** will prevent the timer from moving forward and calling any functions.

The library does not do any dynamic memory allocation.

The number of concurrent tasks is a compile time constant, meaning there is a limit to the number of concurrent tasks. The **in / at / every** functions return **NULL** if the *Timer* is full.

A *Task* value is valid only for the timer that created it, and only for the lifetime of that timer.

Change the number of concurrent tasks using the *Timer* constructors. Save memory by reducing the number, increase memory use by having more. The default is **TIMER_MAX_TASKS** which is currently 16.

If you find this project useful, [consider becoming a sponsor.](https://github.com/sponsors/contrem)
