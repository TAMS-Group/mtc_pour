#!/usr/bin/env python

import seaborn as sns
import pandas as pd

import subprocess
import io

# get OUT from argv[1], else use default 'out'
import sys
OUT = 'out'
if len(sys.argv) > 1:
    OUT = sys.argv[1]

# SED_CMD=R"s/.*_\(.*\)workers.*took \(.*\)ms to find \(.*\) solution.*/\1,\2,\3/"
SED_CMD=R"s/.*_\(.*\)workers.*_\(.*\)solutions_.*divider:\(.*\)\.log.*took \(.*\)ms to find \(.*\) solution.*best solution \([0-9\.]*\).*/\1,\2,\3,\4,\5,\6/"
CMD=f"""
grep -r "Planning took" {OUT}/ | sed '{SED_CMD}'
"""

proc = subprocess.Popen(CMD, shell=True, stdout=subprocess.PIPE)
output = proc.stdout.read()
df = pd.read_csv(io.StringIO(output.decode('utf-8')), header=None)
df.columns = ['workers', 'required solutions', 'divider', 'time', 'solutions', 'best cost']

print(output.decode('utf-8'))
print(df)

# add 'time / solutions' column to the dataframe

# df['time/solutions'] = df['time'] / df['solutions']
df['time(s)'] = df['time'] / 1000

df.to_json(f'{OUT}/df.json', orient='records')

# make 0 inf workers infinite
import math
df['workers'] = df['workers'].replace(0, math.inf)

# plot stripplots for all three columns into a figure

import matplotlib.pyplot as plt

fig, axs = plt.subplots(1, 2, figsize=(15, 5))

# # tics centered on buckets
# sns.histplot(x='workers', data=df, ax=axs[0], bins=range(-1,13))
# axs[0].set_title('workers')

# sns.scatterplot(y='time(s)', x='workers', data=df,  ax=axs[1])
# axs[1].set_title('time(s)')
# mean/variance plot instead:

sns.stripplot(x='workers', y='time(s)', data=df, ax=axs[0])
axs[0].set_title('time(s)')

sns.stripplot(x= 'workers', y='solutions', data=df, ax=axs[1])
axs[1].set_title('solutions')

# sns.scatterplot(y='time/solutions', x='workers', data=df, ax=axs[3])
# axs[3].set_title('time/solutions')
# mean/variance plot instead:

# sns.stripplot(x='workers', y='time/solutions', data=df, ax=axs[2])
# axs[2].set_title('time/solutions')

# rotate x labels

for ax in axs:
    for label in ax.get_xticklabels():
        label.set_rotation(90)

# label -1 should be "disabled"

for ax in axs[1:]:
    # set integer labels
    labels = [item.get_text() for item in ax.get_xticklabels()]
    if labels[0] == '-1.0':
        labels[0] = 'seq'
    ax.set_xticklabels(labels)

plt.savefig(f'{OUT}/plot.svg')
plt.show()
