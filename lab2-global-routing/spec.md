# Lab2 - DP based global routing 

**Deadline: 5/20/2026 23:59** 

## Problem Description 
Implement a global router that can complete ISPD 2008 Global Routing Contest [^1], and try to minimize the total overflow (TOF), max overflow (MOF), wirelength (WL) and runtime. 

## Input Format (Parser is provided)
* All benchmarks have multiple metal layers (three-dimensional), with additional congestion information. 
* The input file format specifies the size and shape of the global routing graph, the capacity on edges of the graph, the number of signal nets, and the number of pins and their positions within each net. 
* Example below illustrates a problem with five routing layers. Text in brackets are just comments, they will not be in actual input files.
    ``` 
    grid # # # 
    vertical capacity # # # # # (vertical capacity by default on each layer)
    horizontal capacity # # # # # 
    minimum width # # # # # 
    minimum spacing # # # # # 
    via spacing # # # # # 
    lower_left_x lower_left_y tile_width tile_height 
    num net # 
    netname id_# number_of_pins minimum_width 
    x y layer 
    x y layer 
    [repeat for the appropriate number of nets] 
    # capacity adjustments (to model congestion) 
    column row layer column row layer reduced_capacity_level 
    [repeat for the number of capacity adjustments] 
    ```
### Descriptions: 
* In the vertical and horizontal capacity lines, the first number indicates the capacity on the first layer, the second number indicates the second layer, and so on. 
* Minimum wire widths/spacings are also specified; this impacts capacity calculations. 
* The lower left corner (minimum X and Y) of the global routing region is specified, as well as the width (X dimension) and height (Y dimension) of each tile. 
* Pin positions are given as XY locations, rather than tile locations. 
* Conversion from pin positions to tile numbers can be done with:
$floor(pin\_x - lower\_left\_x) / tile\_width$
$floor(pin\_y - lower\_left\_y) / tile\_height$.
 Pins will **NOT** be on the borders of global routing cells, so there should be no ambiguity. 
* All pins will be within the specified global routing region. 
* Each net will have a minimum routed width; this width will span **all layers**. 
* When routing, compute the utilization of a global routing graph edge by adding the widths of all nets crossing and edge, plus the minimum spacing multiplied by the number of nets. 
* Each wire will require spacing between its neighbors; think of this as having one-half of the minimum spacing reserved on each side of a wire. 
* Example of computing the utilization:
![image](https://hackmd.io/_uploads/r1eEKfXsZl.png)

  If the value of minimum spacing in this layer is 10, then the demand of red grid edge is $40+20+60+10+10+10=150$. 
* Congestion is modeled by including capacity adjustments. Pairs of (adjacent) global routing tiles may have a capacity that is different from the default values specified at the start of a benchmark file. 
* Capacity specified in the $2^{nd}$ and $3^{rd}$ lines is a measure of the available space, not the number of global routing tracks. 
* If the minimum wire width is 20, the minimum spacing 10, and the capacity of a tile is given as 450, this corresponds to 15 minimum width tracks $(15 \times (20+10))$. 
* If a net has more than 1000 sinks (i.e., $\#pins > 1000)$, you **don't** have to route it. 
* If all pins of a net fall in the same tile, you **don't** have to route it. 


## Output Format (handled by given layer assignment algorithm) 
 ```
 netname id # 
 (x1,y1,layer1)-(x2,y2,layer2)
 [repeat for the appropriate number of segments in nets] 
 ! 
 [repeat for the appropriate number of nets] 
 ```
* Only one of {x,y,z} of two endpoints in segments can be different. 
* The {x,y} must be given as XY locations, the conversion from tile numbers to XY locations can be done with:
 $tile\_x \times tile\_width + lower\_left\_x$ 
 $tile\_y \times tile\_height + lower\_left\_y$. 

## Sample input file and output file with illustrations 
* Input: [https://www.ispd.cc/contests/08/3d.txt](https://www.ispd.cc/contests/08/3d.txt) 
* Output: [https://www.ispd.cc/contests/08/3ds1.txt](https://www.ispd.cc/contests/08/3ds1.txt) 
* Description(pdf): [https://www.ispd.cc/contests/08/3d.pdf](https://www.ispd.cc/contests/08/3d.pdf) 

## Suggested Flow 
![image](https://hackmd.io/_uploads/Sk3CYfXsZg.png)

## Benchmarks 
We will run adaptec1, adaptec2, adaptec3, adaptec4, adaptec5, bigblue1, bigblue2, bigblue3, newblue1, newblue2, newblue5, newblue6.


## Router Evaluation
You can use this [perl script](https://www.ispd.cc/contests/08/eval2008.zip) provided from [[1]](https://hackmd.io/fCkL2jC3S469uQ7wk-2gYw?both=&stext=8469%3A10%3A0%3A1775234510%3AYFm3Ix) to check net connectivity and get TOF, MOF, WL of a routing solution.

## Grading 

- Beating the baseline(monotonic routing) get **60%** score.
- The final **40%** will be based on ranking. Note: **Only solutions surpassing the baseline and submitted before last submission date(5/26) will be considered for competition**.

### Ranking
For each benchmark, the results of all students are compared with each other in terms of TOF, MOF, WL, and runtime:
* A less TOF a router has, a higher rank it has. 
* If there is a tie in TOF, MOF is used to break the tie. 
* If MOF is still the same, the routed wire length and runtime are the 2nd tie breaker. 
* The exact quality metric for 2nd tie-breaker is the 
$WL \times (1 + 0.04 \times \log_2(runtime / median\_runtime))$. 

Submissions exceeding the baseline are divided into five tiers:
<center>
    
|Tier| Score Range | Overflow |
|---|---|---|
|4 | 100 | $\leq OF_N$  | 
|3| [99, 90] | $\leq \lfloor \frac{OF_M - OF_N}{1000} \rfloor +OF_N$ | 
|2| [89, 80] | $\leq \lfloor \frac{OF_M - OF_N}{100}\rfloor +OF_N$ |
|1| [79, 70] | $\leq \lfloor \frac{OF_M - OF_N}{10}\rfloor +OF_N$ |
|0| [69, 60] | $\leq OF_M$ |
    
</center>


- $OF_N$ means the overflow of NCTUgr results, please refer to table 1, $OF_M$ means the overflow of Monotonic results, refer to table 2

- Table 1 - Routing result of NCTUgr[^2] results on [1] 

<center>
    
| Benchmarks | TOF | MOF | WL       | Runtime (sec) |
| :--------- | :-- | :-- | :------- | :------------ |
| adaptec1   | 0   | 0   | 5820849  | 28            |
| adaptec2   | 0   | 0   | 5407474  | 21            |
| adaptec3   | 0   | 0   | 13626514 | 52            |
| adaptec4   | 0   | 0   | 12310042 | 42            |
| adaptec5   | 0   | 0   | 16619509 | 89            |
| bigblue1   | 0   | 0   | 6499329  | 47            |
| bigblue2   | 28  | 2   | 9497076  | 57            |
| bigblue3   | 0   | 0   | 13372234 | 50            |
| newblue1   | 492 | 2   | 4810928  | 48            |
| newblue2   | 0   | 0   | 7663497  | 25            |
| newblue5   | 0   | 0   | 24233695 | 117           |
| newblue6   | 0   | 0   | 19324723 | 87            | 

</center>

- Table 2 - Routing result of monotonic routing
<center>

| Benchmarks | TOF    | MOF  | WL       | Runtime (sec) |
| :--------- | :----- | :--- | :------- | :------------ |
| adaptec1   | 164748 | 32   | 5394328  | 14            |
| adaptec2   | 137386 | 54   | 5239523  | 8             |
| adaptec3   | 298718 | 26   | 13336072 | 38            |
| adaptec4   | 78376  | 24   | 12285419 | 19            |
| adaptec5   | 529776 | 56   | 15544885 | 40            |
| bigblue1   | 248048 | 26   | 5567768  | 17            |
| bigblue2   | 169424 | 26   | 9029417  | 21            |
| bigblue3   | 140016 | 32   | 13168380 | 23            |
| newblue1   | 37056  | 22   | 4604622  | 8             |
| newblue2   | 17890  | 30   | 7647296  | 13            |
| newblue5   | 569808 | 56   | 23123401 | 52            |
| newblue6   | 451384 | 50   | 17703722 | 41            |

</center>

- Your final score for each case is determined by your assigned Tier ($T$), the maximum available score points for ranking ($ScorePt$), and your relative rank within that tier.
Let $n$ be the total number of students in your tier, and $k$ be your rank within that same tier (e.g., $k=0$ for the best performance, and $k=n-1$ for the worst). 
The formula is:
$$FinalScoreForEachCase = 60 + 10 \times \min\left(4,\left( T + 0.9\times \frac{n - k}{n} \right)\right)$$


  
- <span style="color:red;">The grading criteria may be adjusted depending on the situation to help raise students' scores.<span>



#### Final Socre
- $FinalScore = Average(FinalScoreInEachCases)$ 

## Provided Files 
* `main.cpp` (Sample code that can generate 3ds1.txt) 
* `ispdData.h` (Data structure for ISPDParser) 
* `LayerAssignment.h` / `LayerAssignment.cpp` (Part of layer assignment code of NCTUGR [^2]) 
* `Makefile` (Sample Makefile)
* `verifier.py` (The evaluation script) 
* `3d.txt` (The sample input) 


## Submission 
Zip all the source codes and a Makefile in `<student_id>.zip` without any folders like below, then submit the .zip file to E3 by the deadline. 
* `<student_id>.zip/` 
    * `main.cpp`
    * `Makefile` (you can submit your version of Makefile)
    * `<Any source codes and necessary files>` 
    * `<We will run your code here>` 
* **Late submission:** 
    - Score * 0.95 before 5/27,
    - Score * 0.8 after 5/27 and before 6/3. 
    - After 6/3 you will get 0 point. 
* Hard coding the output based on input is not allowed.
* Any work by fraud will absolutely get 0 point.
## Scoreboard
- To make the rankings more exciting, we’ve set up Kaggle competitions! You can upload result to Kaggle by running `verifier.py` with argument `--submit` and see everyone else’s performance in real-time.
<!-- - To make the rankings more exciting, we’ve set up Kaggle competitions [Testcase1](https://www.kaggle.com/t/be94fe676f514d7cb67f15c4449b0c5c) and [Testcase2](https://www.kaggle.com/t/4cb7bf598277404490a19a26c36c07ec) ! You can upload your results and see everyone else’s performance in real-time.-->

- Please note: The Kaggle leaderboard is for reference only. **Do not falsify your scores**. Your final grade will be based on the TA's official execution of your final submission.
    
- For quick ranking purposes only, the score is shown as $TOF + MOF \times 10^{-3} + WL \times 10^{-12}$.


- Please use your **Student ID** as team name to join the Kaggle competition.
    
- To automate result uploads, we've added a submission script to the verifier. Please setup your Kaggle token by following the [linked tutorial](https://github.com/Kaggle/kaggle-cli/blob/main/docs/README.md) to enable this functionality.
    
- **Please join the Kaggle competition first; otherwise, your submission will fail.**

- Kaggle scoreboard:
    * adaptec1: https://www.kaggle.com/t/820ace61fdaf4f7fb1cec02d623df5a4
    * adaptec2: https://www.kaggle.com/t/5ded756ccbb94b5eb7963102c6a91faf
    * adaptec3: https://www.kaggle.com/t/cc99210ec66542ca83db581302d3bf7f
    * adaptec4: https://www.kaggle.com/t/ab55305464f046ed993adde7fa557b84
    * adaptec5: https://www.kaggle.com/t/bad8d994cf7f4e9eb0c4a21bfd86f56e
    * bigblue1: https://www.kaggle.com/t/fab52a64a73c4e978692db89d6fae248
    * bigblue2: https://www.kaggle.com/t/bddef136694d4b829276fe8f40b07a4c
    * bigblue3: https://www.kaggle.com/t/10e1782bcfab4a8eb7a0ee8ec7933496
    * newblue1: https://www.kaggle.com/t/d4f4222b45c94a7ab5c3003a57281b11
    * newblue2: https://www.kaggle.com/t/cc801f6aa98743a394d0221f59cbb79d
    * newblue5: https://www.kaggle.com/t/bcf11f1364524148b98bfd1cc00a8899
    * newblue6: https://www.kaggle.com/t/f39ac8551a6f490a9ec1432b7f7c949b



## Executing Procedure 
1. `unzip <student_id>.zip` 
2. `make` 
    * If fail or "Makefile" not found or compile error → 0 point in all benchmarks 
3. `./router <inputFile> <outputFile>` 
    * If "router" is not found or cannot be executed → 0 point in all benchmarks 
    * If runtime limit (default 30mins) is reached → 0 point in this benchmark 
    * If the command returned non-zero exit code (unhandled C++ exceptions, segmentation fault, out of memory, ...) → 0 point in this benchmark 
4. `python verifier.py <inputFile> <outputFile> [--submit] [--competition COMPETITION] [-m MESSAGE]`
    * `--submit` is used to submit your score to Kaggle.

    * `--competition` is only needed if you have changed the .gr file name or default file name is correct.

    * `-m` is used to add a comment to your submission; the default message is "v1".

## Appendix 

### Testing environment
* OS: CentOS 8 
* CPU: Intel(R) Xeon(R) Gold 5220R CPU @ 2.20GHz 48 Cores 96 Threads 
* Memory: 256G 
* GCC version: 8.5.0

---





## Contact Information
If you have any questions regarding the lab, please feel free to contact the TAs.

- Please use the E3 platform email system.
- You MUST **CC all TAs** in your email to ensure a timely response.

## Q&A
Before asking a question, please check this section to see if it has already been answered. This section will be updated dynamically.
    
- **Q1:** get error
    
    ```
    403 Client Error: Forbidden for url: https://api.kaggle.com/v1/competitions.CompetitionApiService/StartSubmissionUpload
    ```
    when submit result to Kaggle.
  **A1:** Please join the Kaggle competition first.
- **Q2:** The layers in `3d.txt` are 1-based, but the sample input in the document is 0-based. Which one should we follow?
  **A2:** Please use `3d.txt` as the primary reference for <span style="color:red;">1-based</span> layer representation.

## References

[^1]: [ISPD 2008 Global Routing Contest](https://www.ispd.cc/contests/08/ispd08rc.html)
[^2]: W.-H. Liu, W. -C. Kao, Y. -L. Li and K. -Y. Chao, ["NCTU-GR 2.0: Multithreaded Collision-Aware Global Routing With Bounded-Length Maze Routing,"](https://ieeexplore.ieee.org/document/6504553) in IEEE Transactions on Computer-Aided Design of Integrated Circuits and Systems, vol. 32, no. 5, pp. 709-722, May 2013, doi: 10.1109/TCAD.2012.2235124. 

<!-- [^2]: W.-H. Liu, Y. -L. Li and C. -K. Koh, ["A fast maze-free routing congestion estimator with hybrid unilateral monotonic routing,"](https://ieeexplore.ieee.org/document/6386752) 2012 IEEE/ACM International Conference on Computer-Aided Design (ICCAD), San Jose, CA, USA, 2012, pp. 713-719.  -->

