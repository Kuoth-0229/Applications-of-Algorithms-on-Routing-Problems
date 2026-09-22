# Lab3 - Routing with solver

**Deadline:** 6/13/2026 23:59

> **Edit History**
> 2026/5/26 13:22 clearify late submission deadline (6/20 23:59)
> 2026/5/25 18:30 add bonus testcase Kaggle
> 2026/5/25 03:25 add preprocess rule

## Problem description
Implement a grid-based 2-pin net router based on the routing part mentioned in the [MCell paper](https://ieeexplore.ieee.org/document/9256556). The goal is to complete all routing under the specified routing boundary. You need to output your clauses based on the parts mentioned in the paper to MaxSAT to complete the routing.

## Input/Output Format

### Input
```
max_layer #
boundary # # # #
min_pitch_size #
via_cost #
nets # 
<netname source_x source_y source_layer target_x target_y target_layer> 
...
```
* `max_layer`: the maximum layer you can use, 0-indexed. That is, you can use layer 0 to #layer - 1
* `boundary`: routing boundary (min x, min y, max x, max y coordinate)
* `min_pitch_size`: the minimum pitch size for the gridline
* `via_cost`: one via equal to how much wirelength. If a via pass 3 layer, it would count twice.
* `nets`: The following # lines are the coordinates of the pins that need to be routed

All inputs can be stored as 32-bit integers, and the boundary and pitch size of each layer are the same.

### Output

```
x_coors #  
# # ....# 
y_coors #  
# # ....# 
<netname1> #
<x y layer>
<x y layer>
... 
<x y layer>
<netname2> #
<x y layer>
<x y layer>
...
```
- `x_coors`: The x-coordinates of the grid lines you draw, followed by # grids, which will be the actual coordinates of your grid lines.
- `y_coors`: The y-coordinates of the grid lines you draw, followed by # grids, which will be the actual coordinates of your grid lines.
- `netname`: The following # lines are grids of the net passing.
- `<x y layer>`: routing results - from the first grid to the last one.


Any nodes that are not positioned on the grid nodes **will be regarded as failures**. When constructing the mesh structure, the pin coordinates can be used as the reference line.

The provided output file represents the routing result. Points will not be awarded if the net exhibits spill-over, short circuits, cycles, or crossings. It is important to note that **a net positioned with its center on the boundary is not classified as a spill-over.**


## Appendix

The solver that can be used for this assignment is [Open-WBO MaxSAT](https://github.com/sat-group/open-wbo).

After choosing the solver to use, you can set the command you want to execute through the `Router::getSysCommand()` part of the return.

<!--
## Verfier
We provide a verifier to check the correctness of your program.

```
./verifier [output.txt] [input.txt]
```

If the terminal shows ```Your routing result is correct```, it means your result is correct. The information also includes your track count and total cost (including via cost), which are the basis of our grading. If there is any issues, please contact TA.-->

## Plotter

```
python3 plotter.py [output.txt]
```

We provide a Python plotter that can help you debug your codes with the visualized routing result. There are two examples in different angles. You can check the detail with `plt.show()`. The second one has some detours that can be refined. The hidden case would not be harder than the second one (about 20 net).

<img style="width:75%" src="https://hackmd.io/_uploads/SkU-fhVyfg.png"/>

<img style="width:75%" src="https://hackmd.io/_uploads/BkCZM2E1zx.png"/>

## Hint
Generally, SAT solvers require describing input files in CNF (Conjunctive Normal Form), which can be simply regarded as multiple Boolean variables or consisting of clauses with OR operators and finally combined with AND operators.
For example: `(x1 OR NOT x2) AND (x2 OR x3 OR x4)`

For connectivity between nets, the main consideration is that each pin must be connected by one edge, and each non-pin grid node must use none or two edges.

Between different nets, a grid node cannot be used by two different nets, except for storing the grid node with Boolean values to indicate whether the node is used or not. <span style="color:red;">**Alternatively, you can focus on those edges which cannot be used at the same time. Then, apply XOR operation on them.**</span>

If the above two conditions are satisfied, the found routing result should be legal.

There is an example for one net on a 2x2 mesh structure. And there are two pins:
`e1`, `e2`, `e3`, `e4`
<img style="width:30%" src="https://hackmd.io/_uploads/HklQfnNkGe.png"/>

For the blue pin, it should select one edge at least:
`(e1 or e4)`
And not more than one edge:
`(-e1 or -e4)`

For the red pin:
`(e2 or e3) or (-e2 or -e3)`

You can choose both e1 and e2 at the same time, or choose not to choose either:
`(e1 or -e2) and (-e1 or e2)`
And,
`(e3 or -e4) and (-e3 or e4)`

How about 3, 4, 5 or even 6 edge?
Putting them into the solver, we can get the result:
```
OPTIMUM FOUND
1 2 -3 -4
```
That is, it identifies the path which is e1 and e2.

## Executing Procedure
We'll use `make` command to build your code, please make sure your Makefile can generate an executable named `Lab3`
1. `unzip <student_id>.zip` 
2. `make`
3. `./Lab3 [input.txt] [output.txt]`
4. Search for `[output.txt]`, found break → 0 point
5. `./verifier [output.txt] [input.txt]`
6. If fail break → 0 point

## Submission
Please submit the following materials in a `.zip` file without any folders like below to E3 by the deadline, specifying your student ID in the subject field (e.g., `StudentID.zip`):
* `<student_id>.zip/` 
    * Source codes (`.cpp`, `.h` ...)
    * `Makefile`
* **Do NOT submit the `open-wbo` executable.** TAs will place the solver executable directly in the root directory of your unzipped folder during grading.
* **Do NOT submit intermediate or generated files.** Please clean your directory before zipping. Exclude any files generated during compilation or execution (e.g., `*.o`, `*.sat`, `*.png`, `*.txt`, `.cnf`).
* **Do NOT submit testcases.**
* **Verify your compiler version.** Before submitting, please ensure you test your code using the specific compiler version mentioned in the Appendix below to guarantee it compiles and runs without errors. You may use `conda` or similar environment management tools to install the required compiler version.

## Grade
* The execution time of each case is **10 mins**, and exceeding it will be considered a **routing failure**.
* No errors such as spill-over, short, cycle, and crossing are allowed for any case. Otherwise, it is also considered a failure.
* **Multithread** is not allowed.
* Please use the solver (open-wbo SAT) and **do not modify the `main.cpp` file**. However, if you have any issues or other ideas, please feel free to raise them.
* The final routing paths must directly reflect the solver's output. **Post-processing on the solver's output (e.g., applying A\* search to optimize or fix the results) is strictly prohibited.** To optimize your solution for the ranking score, you should focus on tuning the weights, constraints, or formulation within the solver's input file.
* Hardcoding clauses directly for specific inputs is prohibited. For example, you cannot force the solver to route a net on a specific path by manually assigning true to certain edges. Your clause generation must be generalized and entirely dependent on the input parameters rather than tailored to bypass the solver's logic.
- While diverse approaches to formulating the solver's input are welcome, sequential pre-processing or pre-routing methods are strictly prohibited. Your constraints must not depend on the order in which nets are processed.<span id="preprocess"></span>

  - Invalid Example (Not Allowed): Running an initial sequential pattern routing pass and using those preliminary results to restrict the MaxSAT solver's choices (e.g., forcing a net into an upper or lower L-shape based on the routing of previous nets). This introduces order dependency.

  - Valid Example (Allowed): Imposing a global, order-independent constraint, such as restricting all nets to only use L-shapes from the beginning.
* There are 4 given case and 2 hidden case. If you can't finish the bonus case, 96 will be your highest score. Scores on each case:

<center>

| Testcase        | correctness | ranking |
| :-------------- | :---------- | :------ |
| case1           | 5           | -       |
| case2           | 5           | -       |
| case3           | 10          | 5       |
| case4           | 10          | 5       |
| case5           | 15          | 5       |
| hidden1         | 15          | 5       |
| hidden2         | 15          | 5       |
| case6 (bonus)   | 10          | -       |

</center>

* The ranking score is based on the cost ranking `(wirelength + via num * via cost)`, and calculated as follows: `(rank1_cost - yours_cost) / (rank1_cost - ranklast_cost).`

* Late submission: 
    * Score * 0.95 before 6/20 23:59
    * After 6/20 23:59, submission is not allowed.
* Wrong submission format: 10 points punishment.


* <span style="color: red;">Plagiarism of any kind is **strictly prohibited**, including but not limited to copying existing code from the internet or from other students. If plagiarism is detected, both the provider and the recipient will receive 0 points for this assignment.</span>

* **Any work by fraud will absolutely get 0 point.**
* Any violation of the rules mentioned above will result in a **score of 0** for this assignment.

* You may use any AI tools to assist in completing this. 

## Scoreboard


- Please use your **Student ID** as team name to join the Kaggle competition.
* Kaggle scoreboard:
    * [case1](https://www.kaggle.com/t/bbe6c3b9a0774129babf71a552a6dc51)
    * [case2](https://www.kaggle.com/t/f0e698b67d19495cb093f97ac4e2f476)
    * [case3](https://www.kaggle.com/t/80a966907ae34aeab22cc764eb6fbb43)
    * [case4](https://www.kaggle.com/t/4331842131d8477980ff0f69f940a188)
    * [case5](https://www.kaggle.com/t/e833c06c1db4495fa0e27f792aef737f)
    * [bonus](https://www.kaggle.com/t/f9d2ca7c8d6242a69dbc92dcc3422c9b)
- Please note: The Kaggle leaderboard is for reference only. **Do not falsify your scores**. Your final grade will be based on the TA's official execution of your final submission.

## Appendix 

### Testing environment
* OS: CentOS 8 
* CPU: Intel(R) Xeon(R) Gold 5220R CPU @ 2.20GHz
* Memory: 256G 
* **GCC version: 8.5.0**


## Contact Information
If you have any questions regarding the lab, please feel free to contact the TAs.

- Please use the E3 platform email system.
- You MUST **CC all TAs** in your email to ensure a timely response.

## Q&A
Before asking a question, please check this section to see if it has already been answered. This section will be updated dynamically.

* **Q1:** I encounter the error `./open-wbo: error while loading shared libraries: libgmp.so.3: cannot open shared object file: No such file or directory`. How do I fix this?
  **A1:** This error occurs because the `libgmp` library is missing from your system. If you are using `conda`, you can install it by running `conda install libgmp` and then updating your library path with `export LD_LIBRARY_PATH="$CONDA_PREFIX/lib"`. Rest assured, you don't need to worry about this for grading—we will ensure the correct environment is set up when verifying your code.
