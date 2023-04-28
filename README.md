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
<table>
  <thead>
    <tr>
      <th style="text-align:center">Rank</th>
      <th style="text-align:center">Percentage of Completion(%)</th>
      <th style="text-align:center">Time consumption(sec)</th>
      <th style="text-align:center">Group number</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="text-align:center">1</td>
      <td style="text-align:center">100</td>
      <td style="text-align:center">4.8777</td>
      <td style="text-align:center">7</td>
    </tr>
    <tr>
      <td style="text-align:center">2</td>
      <td style="text-align:center">100</td>
      <td style="text-align:center">4.9119</td>
      <td style="text-align:center">9</td>
    </tr>
    <tr>
      <td style="text-align:center">3</td>
      <td style="text-align:center">100</td>
      <td style="text-align:center">5.185</td>
      <td style="text-align:center">12</td>
    </tr>
    <tr>
      <td style="text-align:center">4</td>
      <td style="text-align:center">100</td>
      <td style="text-align:center">7.266</td>
      <td style="text-align:center">6</td>
    </tr>
    <tr>
      <td style="text-align:center">5</td>
      <td style="text-align:center">100</td>
      <td style="text-align:center">10.7061</td>
      <td style="text-align:center">3</td>
    </tr>
    <tr>
      <td style="text-align:center">6</td>
      <td style="text-align:center">100</td>
      <td style="text-align:center">16.4615</td>
      <td style="text-align:center">1</td>
    </tr>
    <tr>
      <td style="text-align:center">7</td>
      <td style="text-align:center">100</td>
      <td style="text-align:center">28.9013</td>
      <td style="text-align:center">11</td>
    </tr>
    <tr>
      <td style="text-align:center">8</td>
      <td style="text-align:center">100</td>
      <td style="text-align:center">34.1071</td>
      <td style="text-align:center">10</td>
    </tr>
    <tr>
      <td style="text-align:center">9</td>
      <td style="text-align:center">100</td>
      <td style="text-align:center">49.0447</td>
      <td style="text-align:center">8</td>
    </tr>
    <tr>
      <td style="text-align:center">10*</td>
      <td style="text-align:center">100</td>
      <td style="text-align:center">50.3462</td>
      <td style="text-align:center">Our Team</td>
    </tr>
    <tr>
      <td style="text-align:center">11</td>
      <td style="text-align:center">100</td>
      <td style="text-align:center">68.3962</td>
      <td style="text-align:center">4</td>
    </tr>
    <tr>
      <td style="text-align:center">12</td>
      <td style="text-align:center">100</td>
      <td style="text-align:center">260.4474</td>
      <td style="text-align:center">5</td>
    </tr>
  </tbody>
</table>

+ **For task2:**
<table>
  <thead>
    <tr>
      <th style="text-align:center">Rank</th>
      <th style="text-align:center">Percentage of Completion(%)</th>    
      <th style="text-align:center">Average t_score(points)</th>
      <th style="text-align:center">Group number</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="text-align:center">1</td>
      <td style="text-align:center">100</td>
      <td style="text-align:center">168.706</td>
      <td style="text-align:center">12</td>
    </tr>
    <tr>
      <td style="text-align:center">2*</td>
      <td style="text-align:center">100</td>
      <td style="text-align:center">278.608</td>
      <td style="text-align:center">Our Team</td>
    </tr>
    <tr>
      <td style="text-align:center">3</td>
      <td style="text-align:center">100</td>
      <td style="text-align:center">280.08</td>
      <td style="text-align:center">7</td>
    </tr>
    <tr>
      <td style="text-align:center">4</td>
      <td style="text-align:center">100</td>
      <td style="text-align:center">297.528</td>
      <td style="text-align:center">6</td>
    </tr>
    <tr>
      <td style="text-align:center">5</td>
      <td style="text-align:center">100</td>
      <td style="text-align:center">311.648</td>
      <td style="text-align:center">9</td>
    </tr>
    <tr>
      <td style="text-align:center">6</td>
      <td style="text-align:center">100</td>
      <td style="text-align:center">341.856</td>
      <td style="text-align:center">10</td>
    </tr>
    <tr>
      <td style="text-align:center">7</td>
      <td style="text-align:center">100</td>
      <td style="text-align:center">436.214</td>
      <td style="text-align:center">4</td>
    </tr>
    <tr>
      <td style="text-align:center">8</td>
      <td style="text-align:center">100</td>
      <td style="text-align:center">598.408</td>
      <td style="text-align:center">3</td>
    </tr>
    <tr>
      <td style="text-align:center">9</td>
      <td style="text-align:center">100</td>
      <td style="text-align:center">734.988</td>
      <td style="text-align:center">8</td>
    </tr>
    <tr>
      <td style="text-align:center">10</td>
      <td style="text-align:center">76.356</td>
      <td style="text-align:center">-</td>
      <td style="text-align:center">5</td>
    </tr>
    <tr>
      <td style="text-align:center">11</td>
      <td style="text-align:center">5.97218</td>
      <td style="text-align:center">-</td>
      <td style="text-align:center">11</td>
    </tr>
    <tr>
      <td style="text-align:center">12</td>
      <td style="text-align:center">0.11138</td>
      <td style="text-align:center">-</td>
      <td style="text-align:center">1</td>
    </tr>
  </tbody>
</table>

## 3. Documentation
+ [Report of Ziqi Han](https://github.com/MRHan-426/MPC-BicycleModel/blob/master/doc/report_ziqihan.pdf)
+ [Report of Siyuan Yin](https://github.com/MRHan-426/MPC-BicycleModel/blob/master/doc/report_siyuanyin.pdf)
+ [Report of Qilin He](https://github.com/MRHan-426/MPC-BicycleModel/blob/master/doc/report_qilinhe.pdf)
+ [Report of Yifan Wang](https://github.com/MRHan-426/MPC-BicycleModel/blob/master/doc/report_yifanwang.pdf)
+ [Report of Yuzhou Chen](https://github.com/MRHan-426/MPC-BicycleModel/blob/master/doc/report_yuzhouchen.pdf)

## 4. Project Log & Todo List
    
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

