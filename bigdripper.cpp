#include <math.h>
#include <util/delay.h>
#include <WProgram.h>

#define UVLED1_PIN          6 
#define UVLED2_PIN          13 
#define PUMP1_PIN           5
#define PUMP2_PIN           4
#define PUMP3_PIN           3
#define PUMP4_PIN           2
#define PUMP5_PIN           17
#define PUMP6_PIN           16
#define PUMP7_PIN           15
#define PUMP8_PIN           14

#define PUMP_COUNT          8
#define DEVICE_COUNT        9
#define MODE_COUNT          5

#define SHIFT_OFFSET_TYPE   0
#define ON_OFFSET_TYPE      1
#define OFF_OFFSET_TYPE     2

#define PROMPT_ENABLE       0

// Helper macros for frobbing bits
#define bitset(var,bitno) ((var) |= (1 << (bitno)))
#define bitclr(var,bitno) ((var) &= ~(1 << (bitno)))
#define bittst(var,bitno) (var& (1 << (bitno)))

const int _pump_map[] = {PUMP1_PIN, PUMP2_PIN, PUMP3_PIN, PUMP4_PIN,
                            PUMP5_PIN, PUMP6_PIN, PUMP7_PIN, PUMP8_PIN};

/******************************************************************************
 ** Pin
 ******************************************************************************/

class Pin
{
public:
    void init(unsigned char pin, 
                unsigned char step_on = 0, unsigned char step_off = 0, 
                bool enable = false)
    {
        _counter = 0;
        _pin = pin;
        _enable = enable;
        pinMode(_pin, OUTPUT);
        set_step_on(step_on);
        set_step_off(step_off);
        off();
    }

    void inline off() 
    {
        digitalWrite(_pin, LOW);
        if(_tie_pin)
            digitalWrite(_tie_pin, LOW);
        _state = LOW;
    }

    void inline on()
    {
        digitalWrite(_pin, HIGH);
        if(_tie_pin)
            digitalWrite(_tie_pin, HIGH);
        _state = HIGH;
    }

    void disable() 
    { 
        _enable = false; 
        off(); 
    }
    void enable() 
    { 
        _enable = true; 
    }

    void sync()
    {
        off();
        _counter = 0;
    }

#if PROMPT_ENABLE
    void print()
    {
        Serial.print(" state: ");
        Serial.print(_state, DEC);
        Serial.print(" enabled: ");
        Serial.print(_enable, DEC);
        Serial.print(" step_on: ");
        Serial.print(_step_on, DEC);
        Serial.print(" step_off: ");
        Serial.print(_step_off, DEC);
        Serial.print(" offset: ");
        Serial.print(_step_offset, DEC);
        Serial.print(" counter: ");
        Serial.print(_counter, DEC);
    }
#endif // PROMPT_ENABLE

    void inline step()
    {
        if (!_enable) return;
        _counter++;
        //if((_state) && (_counter >= (_step_off + _step_offset)))
        if((_state) && (_counter >= _step_off))
        {
            off();
            _counter = 0;
        } else
        if ((!_state) && (_counter >= (_step_on + _step_offset)))
        {
            on();
            _counter = 0;
        }
    }

    /* setters */
    void set_step_on(unsigned int step) { _step_on = step; }
    void set_step_off(unsigned int step) { _step_off = step; }
    //void set_offset(int offset) { _step_offset = (offset + 1) / 2; }
    void set_offset(int offset) { _step_offset = offset; }
    void reset_offset() { _step_offset = 0; }
    void tie_pin(unsigned char pin) { _tie_pin = pin; }
    void set_step(unsigned int step_on, unsigned int step_off) 
    { 
        set_step_on(step_on);
        set_step_off(step_off);
    }
    
    /* getters */
    unsigned char get_step_on() const { return _step_on; }
    unsigned char get_step_off() const { return _step_off; }
    unsigned char get_state() const { return _state; }
    
private:
    unsigned int _counter;
    unsigned int _step_on;
    unsigned int _step_off;
    unsigned int _step_offset;
    unsigned char _pin;
    unsigned char _tie_pin;
    bool _enable;
    bool _state;
};

/******************************************************************************
 ** PinSet
 ******************************************************************************/

class PinSet
{
public:
    PinSet()
    {
        for(unsigned char idx = 0; idx < PUMP_COUNT; ++idx) 
            _pumps[idx].init(_pump_map[idx]);
        _led.init(UVLED1_PIN);
        _led.tie_pin(UVLED2_PIN);
    }

    void pumps_enable()
    {
        for(unsigned char idx = 0; idx < PUMP_COUNT; ++idx) 
            _pumps[idx].enable();
    }

    void led_enable()
    {
        _led.enable();
    }

    void enable()
    {
        pumps_enable();
        led_enable();
    }

    void pumps_disable()
    {
        for(unsigned char idx = 0; idx < PUMP_COUNT; ++idx) 
            _pumps[idx].disable();
    }

    void disable_led()
    {
        _led.disable();
    }

    void disable()
    {
        pumps_disable();
        disable_led();
    }

    void reset()
    {
        set_default_timings();
        for(unsigned char idx = 0; idx < PUMP_COUNT; ++idx) 
            _pumps[idx].reset_offset();
        _led.reset_offset();
        bitclr(TIMSK1, OCIE1A);
        for(unsigned char idx = 0; idx < PUMP_COUNT; ++idx) 
        {
            _pumps[idx].sync();
        }
        _led.sync();
        bitset(TIMSK1, OCIE1A);
    }

    void set_default_timings()
    {
        led_set_step(130, 26);
        _pumps[0].set_step(108, 48);
        _pumps[1].set_step(106, 50);
        _pumps[2].set_step(106, 50);
        _pumps[3].set_step(108, 48);
        _pumps[4].set_step(106, 50);
        _pumps[5].set_step(101, 55);
        _pumps[6].set_step(104, 52);
        _pumps[7].set_step(98, 58);
        _pumps[8].set_step(134, 22);
    }

    void pump_set_step(unsigned int _on, unsigned int _off)
    {
        for(unsigned char idx = 0; idx < PUMP_COUNT; ++idx) 
        {
            _pumps[idx].set_step_on(_on);
            _pumps[idx].set_step_off(_off);
        }
    }

    void led_set_step(unsigned int _on, unsigned int _off)
    {
        _led.set_step_on(_on);
        _led.set_step_off(_off);
    }

    void prime(unsigned long _delay)
    {
        disable();
        reset();
        pump_set_step(100, 100);
        enable();
        delay(_delay);
        reset();
    }

    void inline step()
    {
        for(unsigned char idx = 0; idx < PUMP_COUNT; ++idx)
        {
            _pumps[idx].step();
        }
        _led.step();
    }

    Pin &operator[] (unsigned char idx)
    {
        if (idx < PUMP_COUNT)
            return _pumps[idx];
        return _led;
    }

private:
    Pin _pumps[PUMP_COUNT];
    Pin _led;
};

/******************************************************************************
 ** Ramp
 ******************************************************************************/

class Ramp
{
public:
    void init(int pt1, int pt2, unsigned long ttl)
    {
        _enable = true;
        _value = 0;
        set_ramp(pt1, pt2, ttl);
    }

    void set_ramp(int pt1, int pt2, unsigned long ttl)
    {
        _value = 0;
        _pt1 = pt1;
        _pt2 = pt2;
        _ttl = ttl;
        _slope = static_cast<float>(_pt2 - _pt1) / static_cast<float>(_ttl);
        _timestamp = millis();
    }

    int step()
    {
        if (_enable) 
        {
            if (timeout())
            {
                _value = _pt2;
            } else
            {
                _value = (millis() - _timestamp) * _slope + _pt1;
            }
        }
        return _value;
    }

    void flip() { set_ramp(_pt2, _pt1, _ttl); }
    void enable() { _enable = true; }
    void disable()  { _enable = false; }
    bool timeout() { return ((millis() - _timestamp) >= _ttl); }
    int get_value() { return _value; }

private:
    int _pt1;
    int _pt2;
    int _value;
    unsigned long _ttl;
    unsigned long _timestamp;
    float _slope;
    bool _enable;
};


/******************************************************************************
 ** Mode
 ******************************************************************************/

class Mode
{
public:
    void init(PinSet *pinset)
    {
        _pinset = pinset;
        _start_ts = millis();
        _pinset->reset();
        _last_step = 0;
        _ttl_time = 0;
        _ttl_step = 0;
        _cnt_step = 0;
        _interval_step = 0;
        init_core();
    }

    bool timeout()
    {
        if (_ttl_time)
        {
            return (millis() > (_start_ts + _ttl_time));
        } else
        {
            return (_cnt_step >= _ttl_step);
        }
    }

    void step()
    {
        unsigned long now = millis();
        if((now - _last_step) > _interval_step)
        {
            step_core();
            _last_step = now;
            _cnt_step += 1;
        }
    }

    virtual void init_core() = 0;
    virtual void step_core() = 0;
    virtual void cleanup() {};

protected:
    unsigned long   _ttl_time;
    unsigned char   _ttl_step;
    unsigned char   _cnt_step;
    unsigned long   _interval_step;
    unsigned long   _last_step;
    unsigned long   _start_ts;
    PinSet          *_pinset;
};

/******************************************************************************
 ** PassMode
 ******************************************************************************/

class PassMode: public Mode
{
public:
    virtual void init_core()
    {
        _pinset->pumps_disable();
        _ttl_step = 8 * random(3, 10);
        _interval_step = 100 * random(30, 40);
        _pump_idx = rand() % PUMP_COUNT;

        _down_cursor = 0;
        _down_offset = 0;
        _up_cursor = 7;
        _up_offset = 0;
        _direction = 1;
        _offset_val = random(1, 3);
    }

    void step_core()
    {
        (*_pinset)[_down_cursor].disable();
        (*_pinset)[_up_cursor].disable();

        _up_cursor += _direction ? 1 : -1;
        _down_cursor += (!_direction) ? 1 : -1;

        if (_down_cursor == PUMP_COUNT)
        {
            _direction = !_direction;
            _up_cursor = 0;
            _down_cursor = 7;
            _down_offset = 0;
            _up_offset = 0;
        } else
        if (_up_cursor == PUMP_COUNT)
        {
            _direction = !_direction;
            _up_cursor = 7;
            _down_cursor = 0;
            _down_offset = 0;
            _up_offset = 0;
        }

        _up_offset += _offset_val;
        _down_offset -= _offset_val;

        (*_pinset)[_down_cursor].set_offset(_down_offset);
        (*_pinset)[_up_cursor].set_offset(_up_offset);
        (*_pinset)[_down_cursor].enable();
        (*_pinset)[_up_cursor].enable();
    }

private:
    unsigned char   _pump_idx;
    char            _down_cursor;
    char            _down_offset;
    char            _up_cursor;
    char            _up_offset;
    char            _offset_val;
    bool            _direction;
};



/******************************************************************************
 ** MarqueeRampMode
 ******************************************************************************/

class MarqueeRampMode: public Mode
{
public:
    virtual void init_core()
    {
        _pinset->pumps_enable();
        _ttl_time = 1000 * random(30, 61);
        _interval_step = 100 * random(5, 15);
        _pump_idx = rand() % PUMP_COUNT;
        _direction = true;
        (*_pinset)[_pump_idx].set_offset(get_offset());
    }

    int get_offset()
    {
        int offset = 1;
        return random(0, 2) ? 5 : -5;
    }

    void step_core()
    {
        if( ((_pump_idx == 0) && !_direction) ||
            ((_pump_idx == (PUMP_COUNT - 1)) && _direction) )
        {
            _direction = !_direction;
        }
        (*_pinset)[_pump_idx].reset_offset();
        _pump_idx += _direction ? 1 : -1;
        (*_pinset)[_pump_idx].set_offset(get_offset());
    }

private:
    unsigned char   _pump_idx;
    bool            _direction;
};


/******************************************************************************
 ** CarpetMode
 ******************************************************************************/

class CarpetMode: public Mode
{
public:
    virtual void init_core()
    {
        _pinset->pumps_enable();
        _ttl_time = 1000 * random(30, 61);
        int bnb = random(10, 30);
        unsigned long ramp_ttl = 1000 * random(1, 5);
        _ramp = (Ramp*)malloc(sizeof(Ramp));
        _ramp->init(-bnb, bnb, ramp_ttl);
    }

    void step_core()
    {
        int val = _ramp->step();
        (*_pinset)[-1].set_offset(val);
        if (_ramp->timeout()) _ramp->flip();
    }

    void cleanup() { free(_ramp); }

private:
    Ramp            *_ramp;
};

/******************************************************************************
 ** MarqueeMode
 ******************************************************************************/

class MarqueeMode: public Mode
{
public:
    virtual void init_core()
    {
        _pinset->pumps_disable();
        _ttl_time = 1000 * random(30, 61);
        _interval_step = 100 * random(5, 15);
        _pump_idx = rand() % PUMP_COUNT;
        _direction = true;
        (*_pinset)[_pump_idx].enable();
        _ramp = (Ramp*)malloc(sizeof(Ramp));
        _ramp->init(_interval_step, 50, _ttl_time);
        int loff = random(5, 16);
        _led_offsets[0] = -loff;
        _led_offsets[1] = 0;
        _led_offsets[2] = loff;
        _led_offsets[3] = 0;
        _led_offset_idx = 0;
    }

    void step_core()
    {
        (*_pinset)[_pump_idx].disable();
        if( ((_pump_idx == 0) && !_direction) ||
            ((_pump_idx == (PUMP_COUNT - 1)) && _direction) )
        {
            _direction = !_direction;
        }
        _pump_idx += _direction ? 1 : -1;
        (*_pinset)[_pump_idx].enable();
        _led_offset_idx += 1;
        (*_pinset)[-1].set_offset(_led_offsets[(_led_offset_idx % 4)]);
        _interval_step = _ramp->step();
    }

    void cleanup() { free(_ramp); }

private:
    unsigned char   _pump_idx;
    bool            _direction;
    Ramp            *_ramp;
    int             _led_offsets[4];
    unsigned int    _led_offset_idx;
};

/******************************************************************************
 ** RandomWalkMode
 ******************************************************************************/

class RandomWalkMode: public Mode
{
public:
    virtual void init_core()
    {
        _pinset->pumps_disable();
        _ttl_time = 1000 * random(30, 61);
        _interval_step = 100 * random(5, 15);
        _pump_idx = rand() % PUMP_COUNT;
        (*_pinset)[_pump_idx].enable();
    }

    void step_core()
    {
        unsigned char next_pump = rand() % PUMP_COUNT;
        if(next_pump == _pump_idx)
        {
            next_pump = (_pump_idx + 1) % PUMP_COUNT;
        }
        (*_pinset)[_pump_idx].disable();
        char offset = random(-5, 6);
        (*_pinset)[-1].set_offset(offset);
        (*_pinset)[next_pump].enable();
        _pump_idx = next_pump;
    }

private:
    unsigned char   _pump_idx;
};


/******************************************************************************
 ** Globals
 ******************************************************************************/

PinSet          pins;
MarqueeMode     _marquee_mode_instance;
RandomWalkMode  _random_walk_mode_instance;
CarpetMode      _carpet_mode_instance;
MarqueeRampMode _marquee_ramp_mode_instance;
PassMode        _pass_mode_instance;

/******************************************************************************
 ** ModeRunner
 ******************************************************************************/

class ModeRunner
{
public:
    ModeRunner()
    {
        _modes[0] = &_marquee_mode_instance;
        _modes[1] = &_random_walk_mode_instance;
        _modes[2] = &_carpet_mode_instance;
        _modes[3] = &_marquee_ramp_mode_instance;
        _modes[4] = &_pass_mode_instance;
    }

    void next_mode()
    {
        unsigned char mode_idx = rand() % MODE_COUNT;
        if (_modes[mode_idx] == _mode)
        {
            mode_idx = (mode_idx + 1) % MODE_COUNT;
        }
        _mode = _modes[mode_idx];
        //_mode = _modes[3];
        _mode->init(&pins);
        Serial.print("MODE: ");
        Serial.println(mode_idx, DEC);
    }

    void step()
    {
        _mode->step();
        if (_mode->timeout())
        {
            next_mode();
        }
    }

private:
    Mode*       _modes[MODE_COUNT];
    Mode*       _mode;
};



/******************************************************************************
 ** Setup
 ******************************************************************************/

void setup()
{
    // disable global interrupts
    cli();

    Serial.begin(9600);
    Serial.print("[");
    
    // setup timer1 - 16
    // resonsible for timing the camera after an event
    TCCR1A = 0;
    TCCR1B = 0;
    /*
    // 1:256
    bitset(TCCR1B, CS12);
    */
    // 1:64
    bitset(TCCR1B, CS11);
    bitset(TCCR1B, CS10);

    //bitset(TCCR1B, CS11);
    //bitset(TCCR1B, CS10);
    // select CTC mode
    bitset(TCCR1B, WGM12);
    // start the exposure loop
    OCR1A = 20;
    // enable compare interrupt
    bitset(TIMSK1, OCIE1A);
    Serial.print("timer... ");

    // enable global interrupts
    sei();

    // random seed
    long seed;
    seed = analogRead(4);
    for(int idx = 0; idx < 10; ++idx)
    {
        seed += analogRead(4);
        seed ^= analogRead(4);
    }
    seed = abs(seed);
    randomSeed(seed);
    Serial.print("seed(");
    Serial.print(seed, DEC);
    Serial.print(")... ");

    // priming
    Serial.print("priming ...");
    pins.prime(50);
    Serial.println("]");
    Serial.flush();

    // init
    Serial.println("init done!");
    pins.enable();
}


/******************************************************************************
 ** Serial
 ******************************************************************************/

#if PROMPT_ENABLE
void ReportStatus(void)
{
    Serial.println("");
    for(unsigned char dev = 0; dev < DEVICE_COUNT; ++dev)
    {
        Serial.print("Device: ");
        Serial.println(dev, DEC);
        pins[dev].print();
        Serial.println("");
    }
}

void Prompt(void)
{
  static long v = 0;
  static unsigned char device = 0;
  static unsigned char onoff = 0;

  //Serial.println((Serial.available() ? "Y" : "N"));
  if (Serial.available()) {
    char ch = Serial.read();

    switch(ch) {
      case '0'...'9':
        v = v * 10 + ch - '0';
        break;
      case '-':
        v *= -1;
        break;
      case 'z':
        v = 0;
        break;
      case 'x':
        pins[device].disable();
        break;
      case 'X':
        pins[device].enable();
        break;
      case 'p':
        /* Report on status of system */
        ReportStatus();
        break;
      case 'd':
        device = v;
        v = 0;
        break;
      case 'S':
        pins.reset();
        break;
      case 'q':
        pins.disable();
        break;
      case 'Q':
        pins.enable();
        break;
      case 'o':
        onoff = !onoff;
        break;
      case 'v':
        pins[device].set_step(pins[device].get_step_on() - 1, pins[device].get_step_off() + 1);
        break;
      case 'b':
        pins[device].set_step(pins[device].get_step_on() + 1, pins[device].get_step_off() - 1);
        break;
      case 's':
        if (onoff)
        {
            pins[device].set_step_on(v);
        } else
        {
            pins[device].set_step_off(v);
        }
        v = 0;
        break;
      defalut:
        Serial.println(ch, DEC);
    }
    Serial.println("");
    Serial.print("Value: ");
    Serial.println(v, DEC);
    Serial.print("Device: ");
    Serial.println(device, DEC);
    Serial.print("onoff: ");
    Serial.println(onoff, DEC);
    Serial.print("millis: ");
    Serial.println(millis(), DEC);
    Serial.print("> ");
  }
}
#endif // PROMPT_ENABLE

/******************************************************************************
 ** Main loop
 ******************************************************************************/

void loop()
{
#if PROMPT_ENABLE
    Prompt();
#else 
    ModeRunner _mode_runner;
    _mode_runner.next_mode();
    for (;;)
    {
        _mode_runner.step();
    }
#endif // PROMPT_ENABLE
}


ISR(TIMER1_COMPA_vect) 
{
    pins.step();
}
