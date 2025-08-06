# Welcome and some thoughts
Welcome to the course of Modeling and Control of Robotics (MAE 547).

***



## Why still modeling and control of robots?

In the era of big data and artificial intelligence (AI), it’s easy to assume that traditional mathematical modeling and control of robots have taken a back seat to machine learning and AI. But in robotics—where real-world physics, safety, and precision still matter—understanding how to model and control robots, as  dynamic systems, remains as essential as ever.  Whether you’re building a robot arm, a walking robot, or designing an autonomous vehicle, a principled grasp of modeling and control enables you to predict behavior, ensure stability, and build systems that not only learn—but reason and respond robustly in the physical world. MAE 547 will equip you with these skills, bridging classical techniques with modern robotics applications.

***


## Why a robot arm?
This course is designed to ground you in the foundational tools that make intelligent machines truly reliable: homogeneous transformations, forward and inverse kinematics, Jacobians, statics, dynamics, and feedback control.

We focus on how to mathematically describe and control the motion of robots—particularly robot arms. At first glance, this might seem narrow, especially when you see humanoid robots, animal-inspired quadrupeds, or multi-fingered robotic hands performing impressive feats online. But if you look closely, the underlying building blocks of those systems are remarkably consistent. Each articulated component—whether a leg, an arm, or a finger—can be modeled as a robot arm attached to a mobile base like a pelvis, shoulder, or palm. Mastering the modeling and control of robot arms is therefore not only foundational, but also a critical stepping stone toward designing and understanding more complex robotic systems.

***



## About this online learning materials

This interactive online resource is designed for ASU MAE 547 students, global robotics learners, and industry practitioners who want to learn—or refresh—the fundamentals of robotics. The goal is to present robotics concepts in a concise, accessible format supported by visual aids, linked background references, and Python-based learning modules.

Many of the current materials are adapted from  [Robotics: Modelling, Planning and Control]((https://people.disim.univaq.it/~costanzo.manes/EDU_stuff/Robotics_Modelling,%20Planning%20and%20Control_Sciavicco_extract.pdf)), by Bruno Siciliano, Lorenzo Sciavicco, Luigi Villani, Giuseppe Oriolo, as well as selected content from [Modern Robotics]((https://hades.mech.northwestern.edu/images/7/7f/MR.pdf)) by Kevin M. Lynch and Frank C. Park. These notes are specifically tailored for students in MAE 547 but are also intended as a useful reference for anyone interested in robotics—offering a quick way to learn or review topics without needing to read an entire textbook. When possible, I recommend that students engage with the following robotics books.

- [**Robotics: Modeling, Planning and Control**](https://people.disim.univaq.it/~costanzo.manes/EDU_stuff/Robotics_Modelling,%20Planning%20and%20Control_Sciavicco_extract.pdf)  by Bruno Siciliano, Lorenzo Sciavicco, Luigi Villani,
Giuseppe Oriolo, Springer, 2009

- [**Modern Robotics Mechanics, Planning, and Control**]((https://hades.mech.northwestern.edu/images/7/7f/MR.pdf)) by Kevin M. Lynch and Frank C. Park, Cambridge University Press, 2017

While this site is still under active development, the goal is to continuously expand it with advanced topics such as:
* MuJoCo-based simulation examples
* Richer visualizations and animations
* More tightly integrated Python code

If you have suggestions or spot areas for improvement, feel free to [open an issue](https://github.com/asu-iris/course_robotics/issues) on the GitHub repository.

If you find this material helpful, please consider giving it a star [here](https://github.com/asu-iris/course_robotics). Your feedback is highly appreciated and will help improve this resource for everyone!


***


## About the PI and IRIS lab

I am an Assistant Professor in the Ira A. Fulton Schools of Engineering at Arizona State University. PI of [Intelligent Robotics and Interactive Systems (IRIS)](https://asu-iris.github.io/) Lab.
This is the lab webpage: https://irislab.tech/.
Here are some cool demos showing what we are doing: 

### Dexterous Manipulation

<div style="margin-bottom: 1.5em;">
  <div style="font-weight: bold; margin-bottom: 0.2em;">Real-time planning and control for dexterous manipulation (everything is autonomous)</div>
  <video width="640" height="360" controls>
    <source src="./misc/iris_demo/cf_demo.mp4" type="video/mp4">
    Your browser does not support the video tag.
  </video>
</div>


<div style="margin-bottom: 1.5em;">
  <div style="font-weight: bold; margin-bottom: 0.2em;">Multi-fingered robotic hand (Allegro hand) learns in-hand rotation of objects from scratch </div>
  <video width="640" height="360" controls>
    <source src="./misc/iris_demo/contactSDF-media-learn.mp4" type="video/mp4">
    Your browser does not support the video tag.
  </video>
</div>


<div style="margin-bottom: 1.5em;">
  <div style="font-weight: bold; margin-bottom: 0.2em;"> TwinTrack: Reality to Simulation, Simulation to Reality </div>
  <video width="640" height="360" controls>
    <source src="./misc/iris_demo/twintrack.mp4" type="video/mp4">
    Your browser does not support the video tag.
  </video>
</div>


### Robot learning from humans (feedback and demonstrations)

<div style="margin-bottom: 1.5em;">
  <div style="font-weight: bold; margin-bottom: 0.2em;"> Robot learning skills from YouTube </div>
  <video width="640" height="360" controls>
    <source src="./misc/iris_demo/lfd-llm.mp4" type="video/mp4">
    Your browser does not support the video tag.
  </video>
</div>

<div style="margin-bottom: 1.5em;">
  <div style="font-weight: bold; margin-bottom: 0.2em;"> Robot learning from human physical correction </div>
  <video width="640" height="360" controls>
    <source src="./misc/iris_demo/safe_mpc_align4.mp4" type="video/mp4">
    Your browser does not support the video tag.
  </video>
</div>


