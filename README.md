#  MPC-BicycleModel


<p align="center">
  <img src="https://github.com/MRHan-426/MPC-BicycleModel/blob/master/data/output_example.gif" alt="gif">
</p>

![ROB535](https://img.shields.io/badge/ROB535-Team2-blue)
![Primary language](https://img.shields.io/badge/MATLAB-100.0%25-orange)
![License](https://img.shields.io/badge/license-MIT-green)



This is Team 2's final project git repository for ROB535: Self-Driving Cars. 

In this project, we control a 6-state bicycle to race while avoiding random obstacles using **MPC**.

The team members include: Ziqi Han, Siyuan Yin, Qilin He, Yifan Wang, Yuzhou Chen.

Project details and task requirements can be found [here](https://github.com/MRHan-426/MPC-BicycleModel/blob/master/doc/NAVARCH_565_001_WN_2023_FinalProject.pdf).

---

## 1. Usage of our repo
Add directories **src** and **trajectories** to path.


```shell
execute run.m

```

To generate gif for visualization:
```shell
execute make_gif.m
```



## 2. Result
+ **For task1:**



+ **For task2:**



## 3. Project Log & Todo List
    
- [x] 1. Finish writing PID control for part1.

- [x] 2. Use PID control for part2.

    - [x] 2.1 Debug.
    - [x] 2.2 Test for success rate and time => This method is abandoned.

- [x] 3. Use fmincon for part2.

    - [x] 3.1 Finish writing constraints.
    - [x] 3.2 Debug => This method is abandoned.

- [x] 4. Use MPC for part2.

    - [x] 4.1 Generate reference trajectory for MPC.
    - [x] 4.2 Debug.
    - [x] 4.3 solve QP no solution problem.
    - [x] 4.4 Test for success rate and time.

- [x] 5. Upload and Report.

    - [x] 5.1 Draw trajectory.
    - [x] 5.2 Write Report and Upload code.

- [ ] 6. Use MPPI for part2.

    - [ ] 6.1 Write Cost function.
    - [ ] 6.2 Write MPPI loop.
    - [ ] 6.3 Test for success rate and time.

