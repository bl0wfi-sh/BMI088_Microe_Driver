/*
    Complimentary filter library for estimation of single axis angle on a quadcopter.
    Library is made to fuse accelerometer and gyro data.
    Accel = Angle estimation (low pass filtered)
    Gyro = Angular rate estimation (integrated + high pass filtered)

    Math and theory taken from the following source -> https://robottini.altervista.org/wp-content/uploads/2014/04/filter.pdf
    
    Author      : Felix Blanco
    Create Time : July 2022
    Change Log  :

    The MIT License (MIT)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#ifndef _COMP_H_
#define _COMP_H_

class Complimentary {
public:
    Complimentary();

    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    float updateAngle(float newAngle, float newRate, float dt);
    void setAngle(float angle); // Used to set angle, this should be set as the starting angle

    /* These are used to tune the Complimentary filter */
    void setTao(float t);
    float getTao();

private:
    /* Complimentary filter variables */
    float tao; // Process noise variance for the accelerometer
    float angle; // The angle calculated by the Complimentary filter
};

#endif
