# The ball is the trackball

This is Arduino code and 3D-printable models for a Bluetooth trackball in which all the electronics are inside the ball.

[Demo video.](https://www.youtube.com/watch?v=QbE3H4vglHw)

[Blog post.](https://blog.jfedor.org/2021/01/bluetooth-trackball-mark-ii.html)

It runs on Adafruit's [Feather nRF52840 Sense](https://www.adafruit.com/product/4516) and uses the accelerometer, gyroscope and magnetometer data with a sensor fusion algorithm to get the ball's orientation. When you rotate the ball, it moves the mouse cursor accordingly.

TODO:

* whitelist advertising to only allow connections from bonded devices unless the user requests pairing mode
* go to sleep after a period of inactivity and wake when moved
* OTA firmware updates
