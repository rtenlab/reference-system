# Copyright 2021 Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from bokeh.io import show
from bokeh.models import ColumnDataSource
from bokeh.models import DatetimeTickFormatter
from bokeh.models import NumeralTickFormatter
from bokeh.palettes import viridis
from bokeh.plotting import figure


def summary(memory_data_util, ros2_data_util, size):
    ust_memory_usage_dfs = memory_data_util.get_absolute_userspace_memory_usage_by_tid()
    kernel_memory_usage_dfs = memory_data_util.get_absolute_kernel_memory_usage_by_tid()
    tids = ros2_data_util.get_tids()

    colours = viridis(len(tids) + 1)
    first_tid = min(tids)
    starttime = ust_memory_usage_dfs[first_tid]. \
        loc[:, 'timestamp'].iloc[0].strftime('%Y-%m-%d %H:%M')
    memory = figure(
        title='Memory usage per thread/node',
        x_axis_label=f'time ({starttime})',
        y_axis_label='memory usage',
        plot_width=size, plot_height=size,
    )

    i_colour = 0
    for tid in tids:
        legend = str(tid) + ' ' + str(ros2_data_util.get_node_names_from_tid(tid))
        # Userspace
        memory.line(
            x='timestamp',
            y='memory_usage',
            legend=legend + ' (ust)',
            line_width=2,
            source=ColumnDataSource(ust_memory_usage_dfs[tid]),
            line_color=colours[i_colour],
        )
        # Kernel
        memory.line(
            x='timestamp',
            y='memory_usage',
            legend=legend + ' (kernel)',
            line_width=2,
            source=ColumnDataSource(kernel_memory_usage_dfs[tid]),
            line_color=colours[i_colour],
            line_dash='dotted',
        )
        i_colour += 1

    memory.title.align = 'center'
    memory.legend.label_text_font_size = '11px'
    memory.xaxis[0].formatter = DatetimeTickFormatter(seconds=['%Ss'])
    memory.yaxis[0].formatter = NumeralTickFormatter(format='0.0b')

    show(memory)
