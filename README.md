Portable Robotics Eye Vergence Control
======================================

This software implements a bio-inspired model for the control of vergence eye movements for robotic vision.

http://www.pspc.unige.it/Code/index.html

The software can be used for different purposes:
- for the binocular coordination of a robotic stereo head.
- as test-bed for the study of vergence eye movements,
- as a teaching tool to show vergence behavior and its implications,

The robustness and adaptivity of the bio-inspired approach allows a control that is easily portable on stereo head with different kinematic characteristics, and that is robust to mechanical imprecision, as well as to changeable and unpredictable lighting condition of real environments.

### Dependencies
- [OpenCV](https://github.com/opencv/opencv) >= 3.3.1
- [CMake](https://cmake.org)

### Features
- Easy portability on binocular robotic platforms with different geometries
- Real-time performance
- Effective alignment of the eyes/cameras in the 3D space (error < 0.2 deg)
- Control of both horizontal and vertical alignment of the eyes/cameras
- Software developed in C/C++ with OpenCV 3.3.1
- Cross-platform enabled using CMake

The source code is provided with an application example that loads a stereo pair from the [Genoa Pesto Database](http://www.pspc.unige.it/genuapesto/index.html).
In the demo, the stereo pair is shown in anaglyph mode, and the robot fixation is simulated by the user clicking with the mouse cursor on the desired fixation point.
The vergence control is then computed at fixation and vergence eye movements are simulated shifting the images proportionally to the control.
If the control is working properly, the anaglyph at fixation turns to gray scale.

### Videos
Click on the images below (you will be redirected to Youtube).

||
|:---:|
| The algorithm |
|[![video-1](https://img.youtube.com/vi/k6yDMyht184/maxresdefault.jpg)](https://youtu.be/k6yDMyht184)|

---

||
|:---:|
| The algorithm working on the iCub stereo head |
| [![video-2](http://img.youtube.com/vi/viO-SMzpHxo/maxresdefault.jpg)](https://youtu.be/viO-SMzpHxo) |

### Future work
- Embed the control in a script to work in the iCub Simulator
- Port the code in Python

### Authors
:email: For questions, comments or suggestions, reach out to the authors:
- [**Agostino Gibaldi**](mailto:agostino.gibaldi@gmail.com)
- **Mauricio Vanegas**
- **Andrea Canessa**
- **Silvio P. Sabatini**

### Reference publications
- Gibaldi, A., Vanegas, M., Canessa, A., & Sabatini, S. P. (2017). [A portable bio-inspired architecture for efficient robotic vergence control](https://link.springer.com/article/10.1007/s11263-016-0936-z). International Journal of Computer Vision, 121(2), 281-302.
- Gibaldi, A., Canessa, A., Chessa, M., Sabatini, S. P., & Solari, F. (2011). [A neuromorphic control module for real-time vergence eye movements on the iCub robot head](https://ieeexplore.ieee.org/document/6100861). In Humanoid Robots (Humanoids), 2011 11th IEEE-RAS International Conference on (pp. 543-550).
