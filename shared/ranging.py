"""Pulse timing adapter for the installed SunFounder ultrasonic hardware.
Based on robot_hat.Ultrasonic._read; measure one pulse and allow its tail to end
before retriggering. A completed minimum pulse width proves a range lower bound;
a missing rising edge or a pin stuck high before the trigger is NOT free space.
"""
import time

class RangeReader:
    def __init__(self,sensor,clock=time):
        self.sensor=sensor; self.clock=clock; self.next_trigger=0.
        self.maximum_cm=400.

    def read(self):
        c=self.clock; u=self.sensor
        delay=self.next_trigger-c.monotonic()
        if delay>0: c.sleep(delay)
        if u.echo.value():
            self.next_trigger=c.monotonic()+.07
            return None,'invalid'
        u.trig.off();c.sleep(.001);u.trig.on();c.sleep(.00001);u.trig.off()
        trigger=c.monotonic();self.next_trigger=trigger+.07
        while not u.echo.value():
            if c.monotonic()-trigger>.01: return None,'invalid'
        rise=c.monotonic()
        limit=self.maximum_cm/(u.SOUND_SPEED*50.)
        while u.echo.value():
            if c.monotonic()-rise>=limit: return None,'beyond_range'
        distance=(c.monotonic()-rise)*u.SOUND_SPEED*50.
        if 2<=distance<=self.maximum_cm: return round(distance,2),'valid'
        return None,'invalid'
