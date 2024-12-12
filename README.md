# ITA-CBS2

map_data: [data_download](https://drive.google.com/file/d/1NOI4AxlLeqFZKxPTLKaU5x70LJUBswk-/view?usp=sharing)

ITA-CBS new code
Build:

```bash
mkdir build
cd build
cmake ..
cmake --build . --target CBS
cmake --build . --target CBS_parallel
cmake --build . --target ECBS
cmake --build . --target ECBS_parallel
cmake --build . --target CBS_distributed
cmake --build . --target ECBS_distributed
```

Run (In build dir):
```bash
./CBS -i ../map_file/debug_cbs_data.yaml -o ../outputs/output.yaml
./CBS_parallel -i ../map_file/debug_cbs_data.yaml -o ../outputs/output.yaml
./ECBS -i ../map_file/debug_cbs_data.yaml -o ../outputs/output.yaml
./ECBS_parallel -i ../map_file/debug_cbs_data.yaml -o ../outputs/output.yaml
./CBS_distributed -i ../map_file/debug_cbs_data.yaml -o ../outputs/output.yaml
```

Run Batch Tests:
```bash
cd run_script
#runs cmake and builds targets
python run.py configure
#select target algorithm and run batch tests on its files
python run.py <COMMAND> -i ../map_file/<FILE FOLDER NAME>/ -c -w <NUMBER>
#       -c is to run all of the files in the directory given
#       -w is optional, number can be anywhere from 1.0 to 1.2
#       -t can be used to set the timeout duration in seconds
#       Example: python run.py ECBS-p -i ../map_file/Boston_0_256_020/ -c -w 1.02
```

Run CBS distribute with 2 processes (In build dir)
```bash
mpirun -np 2 ./CBS_distributed -i ../map_file/debug_cbs_data.yaml -o ../outputs/output.yaml
```
work in progress: Run ECBS distribute with 2 processes (In build dir)
```bash
mpirun -np 2 ./ECBS_distributed -i ../map_file/debug_cbs_data.yaml -o ../outputs/output.yaml
```


Generate test case:

```bash
# please check generate_data_for_exp1.py/generate_data_for_exp2.py
# there are some config in it

# to activate python env run this first
source /workspaces/CBS_401/python/myenv/bin/activate 

# in root dir:
python python/generate_data_for_exp2.py --map_path map_file/Boston_0_256.map --output_dir map_file/Paper_boston_256_256_060 --common_ratio 0.6
```

Test:

```bash
# TYMPath and CBSTAPath are just paths to binary file, no difference
# in root dir:
python python/compareTYM2TA.py --TYMPath cmake-build-debug/CBSTA_remake --CBSTAPath cmake-build-debug/ITACBS_remake --map_dir map_file/Paper_boston_256_256_060 --time 15 --seed 0
```

Visualize:

Old:
```bash
# don't use large map(larger than 100*100), it will be very slow and agents will be very small.
# in root dir:
python python/visualize.py [your testcase yaml file] [your program output yaml]
```

New:
You should clone my PlanViz and use ITA-CBS branch:
```bash
python python/PlanViz/script/plan_viz.py --map  map_file/Boston_0_256.map --plan_path_type2 outputs/output.yaml --grid --aid --ca --tid --ppm 2
```


Profiling can be done using Vtune or the default profiling tool in CLion. Please note that if you are on Windows OS, the profiling tool will not work on CLion. Vtune can be used, however, CLion is suggested for this project. Below are the steps to run Vtune. 

Run Vtune Profiling
Peformance Snapshots:
Below is an example 
```
vtune -collect performance-snapshot -r cbs_parallel_snapshot ./CBS_parallel -i ../map_file/debug_cbs_data.yaml -o ../outputs/output.yaml
```
Below saves the report to an html file
```
vtune -report summary -r cbs_parallel_snapshot -format html -report-output cbs_parallel_report.html
```

If you would like to have a more detailed report about which function calls are being made at a significantly higher rate you can also run a hotspots report by replacing `performance-snapshot` with `hotspots`


Thank @MinakoOikawa (twitter id) for providing the correct dynamic hungarian implementation.
