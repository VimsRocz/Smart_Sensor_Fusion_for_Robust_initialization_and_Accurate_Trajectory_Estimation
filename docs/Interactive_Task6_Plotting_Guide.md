# Interactive Task 6 Plotting User Guide


## Overview

The Task 6 plotting functionality has been significantly enhanced with interactive features that allow users to:

- **Zoom and pan** for detailed data exploration
- **Hover tooltips** showing exact values and timestamps
- **Toggle data series** visibility via interactive legends
- **Export plots** in various formats
- **Compare multiple methods and frames** through dashboards

## Python Interactive Plotting

### Features

1. **Interactive Plotly Integration**
   - 3×3 subplot layout (Position, Velocity, Acceleration vs X/Y/Z components)
   - Zoom, pan, and reset functionality
   - Hover tooltips with detailed information
   - Legend toggling to show/hide data series
   - Built-in export tools (PNG, PDF, HTML)

2. **Performance Optimization**
   - Automatic data subsampling for large datasets
   - Configurable maximum points for optimal performance
   - Maintains data integrity while improving responsiveness

3. **Dashboard Creation**
   - Comprehensive overview of all method/frame combinations
   - Easy navigation between different plots
   - Centralized access to all results

### Usage

#### Basic Interactive Plotting

```python
from plot_overlay_interactive import plot_overlay_interactive

plot_overlay_interactive(
    frame="NED",
    method="TRIAD", 
    t_imu=time_imu,
    pos_imu=position_imu,
    vel_imu=velocity_imu,
    acc_imu=acceleration_imu,
    t_gnss=time_gnss,
    pos_gnss=position_gnss,
    vel_gnss=velocity_gnss,
    acc_gnss=acceleration_gnss,
    t_fused=time_fused,
    pos_fused=position_fused,
    vel_fused=velocity_fused,
    acc_fused=acceleration_fused,
    out_dir="results/",
    t_truth=time_truth,
    pos_truth=position_truth,
    vel_truth=velocity_truth,
    acc_truth=acceleration_truth,
    include_measurements=True,
    save_static=True
)
```

#### Using Enhanced task6_plot_truth.py

```bash
# Basic interactive plotting
.venv/bin/python PYTHON/src/task6_plot_truth.py \
    --est-file results/IMU_X002_GNSS_X002_TRIAD_kf_output.mat \
    --truth-file DATA/Truth/STATE_X001.txt \
    --interactive \
    --show-measurements

# Create dashboard with all plots
.venv/bin/python PYTHON/src/task6_plot_truth.py \
    --est-file results/IMU_X002_GNSS_X002_TRIAD_kf_output.mat \
    --truth-file DATA/Truth/STATE_X001.txt \
    --interactive \
    --create-dashboard

# Use static plots only
.venv/bin/python PYTHON/src/task6_plot_truth.py \
    --est-file results/IMU_X002_GNSS_X002_TRIAD_kf_output.mat \
    --truth-file DATA/Truth/STATE_X001.txt \
    --static-only
```

#### Dashboard Creation

```python
from plot_overlay_interactive import create_comparison_dashboard

create_comparison_dashboard(
    results_dir="PYTHON/results/",
    methods=["TRIAD", "Davenport", "SVD"],
    frames=["NED", "ECEF", "Body"]
)
```

### File Outputs

- **Interactive HTML**: `method_frame_interactive.html` (~5-6MB each)
- **Static PNG**: High-resolution export for publications
- **Static PDF**: Vector format for presentations
- **Dashboard HTML**: Navigation interface for all plots


### Features

1. **Enhanced 3×3 Layout**
   - Matches Python implementation for consistency
   - Position, velocity, and acceleration in separate rows
   - X/Y/Z components in separate columns

2. **Interactive Capabilities**
   - Built-in zoom and pan tools
   - Data cursor for precise value inspection
   - Custom keyboard shortcuts (F: fullscreen, G: grid toggle, R: reset)
   - Custom toolbar with reset and grid toggle buttons

3. **Advanced UI Features**
   - Figure state management
   - Interactive datatips with detailed information
   - Mouse and keyboard event handling
   - Multiple export formats (.fig, .pdf, .png)

### Usage

#### Basic Interactive Plotting

```matlab

% Basic usage with interactive features enabled (default)
pdf_path = task6_overlay_plot(est_file, truth_file, method, frame, ...
                             dataset, output_dir, debug, true);

% Disable interactive features (legacy mode)
pdf_path = task6_overlay_plot(est_file, truth_file, method, frame, ...
                             dataset, output_dir, debug, false);
```

#### Direct Interactive Function Call

```matlab
% Create interactive 3x3 plot directly
pdf_path = task6_overlay_plot_interactive(est_file, truth_file, method, ...
                                         frame, dataset, output_dir, debug);
```

#### Example with Real Data

```matlab
% Set up paths and parameters
est_file = 'PYTHON/results/IMU_X002_GNSS_X002_TRIAD_kf_output.mat';
truth_file = 'DATA/Truth/STATE_X001.txt';
method = 'TRIAD';
frame = 'ECEF';
dataset = 'IMU_X002_GNSS_X002';

% Create interactive plot
pdf_path = task6_overlay_plot_interactive(est_file, truth_file, method, ...
                                         frame, dataset, output_dir, false);
```

### Interactive Controls

| Control | Action |
|---------|--------|
| **Mouse** | |
| Click + Drag | Zoom into region |
| Shift + Click + Drag | Pan across data |
| Double-click | Reset zoom/pan |
| Right-click on data | Open context menu |
| **Keyboard** | |
| F | Toggle fullscreen |
| G | Toggle grid on all subplots |
| R | Reset all views |
| **Toolbar** | |
| Zoom tool | Enable zoom mode |
| Pan tool | Enable pan mode |
| Data cursor | Enable data inspection |
| Reset button | Reset all subplot views |
| Grid button | Toggle grid display |

### File Outputs

- **Static PDF**: Vector format for publications  
- **Static PNG**: High-resolution raster format
- **RMSE Analysis**: Separate interactive RMSE plot

## Performance Considerations

### Python
- **Large datasets** (>50k points): Automatic subsampling to ~2000 points
- **Memory usage**: ~5-6MB per interactive HTML plot
- **Loading time**: <5 seconds for typical datasets

- **Figure memory**: Interactive figures use more memory than static plots
- **Performance**: Smooth interaction with datasets up to 100k points


|---------|-----------------|-------------------|
| **Layout** | 3×3 subplots | 3×3 subplots |
| **Interactivity** | Web-based | Desktop application |
| **Zoom/Pan** | ✓ | ✓ |
| **Hover tooltips** | ✓ | ✓ (data cursor) |
| **Export options** | HTML, PNG, PDF | .fig, PNG, PDF |
| **Performance** | Auto-subsampling | Memory-based |
| **Dashboard** | ✓ | Manual |

## Migration Guide

### From Static to Interactive (Python)

Old code:
```python
from plot_overlay import plot_overlay
plot_overlay(frame, method, ..., include_measurements=True)
```

New code (automatic):
```python
from plot_overlay import plot_overlay
plot_overlay(frame, method, ..., include_measurements=True, interactive=True)
```


Old code:
```matlab
pdf_path = task6_overlay_plot(est_file, truth_file, method, frame, dataset, output_dir);
```

New code (automatic):
```matlab
pdf_path = task6_overlay_plot(est_file, truth_file, method, frame, dataset, output_dir, false, true);
```

## Troubleshooting

### Python Issues

**Problem**: `ImportError: Plotly not available`  
**Solution**: Install plotly and kaleido: `python3 -m pip install plotly kaleido`

**Problem**: Large HTML files (>50MB)  
**Solution**: Use data subsampling or reduce max_points parameter

**Problem**: Static export fails  
**Solution**: Install kaleido: `python3 -m pip install kaleido`


**Problem**: Interactive features not working  

**Problem**: Figures not saving  
**Solution**: Check write permissions in output directory

**Problem**: Memory issues with large datasets  
**Solution**: Subsample data before plotting

## Best Practices

1. **Use interactive plots for exploration**, static plots for publications
2. **Subsample large datasets** (>10k points) for better performance  
4. **Create dashboards** for comparing multiple methods/frames
5. **Export to PDF/PNG** for including in reports and papers

## Examples and Testing

See the following test scripts for usage examples:

- `PYTHON/test_interactive_plotting.py` - Basic functionality test
- `PYTHON/test_optimized_interactive.py` - Performance-optimized real data test

