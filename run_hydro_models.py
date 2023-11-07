import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os

steady_input_file = './hydro_steady.i'
transient_input_file = './hydro_osc.i'
results_folder = './out_files/'
exe_file = './moose'
n_cores = 10

steady_command = 'mpiexec -n'+' '+str(n_cores)+' '+exe_file+' '+ '-i' \
                +' '+steady_input_file+' '
os.system(steady_command)

transient_command = 'mpiexec -n'+' '+str(n_cores)+' '+exe_file+' '+ '-i' \
                +' '+transient_input_file+' '
os.system(transient_command)