# Clearance Analyzer

This tool gets clearance measurements along the scan path both upwards and side to side. From this it further picks segments of the file that are considered to be of interest, where interest is determined by overhead clearance. As such tunnels, overpasses, bridges, etc. are selected and have more detailed figures generated for each segment of interest. Figures are saved in the "out" folder, and further sorted by input filename.

It should be noted that the "sideviews" are perspective, that is the plot should be viewed as if you were looking out the window of the vehicle which took the scan.

## Usage

### If Matlab Parallel Toolkit is Not Available

Add the libs folder to your matlab path and then simply run the matlab script get_clearances_octree.m either all at once or section by section. When prompted select a *single* las file you want to process.

### If Matlab Parallel Toolkit is Available

Start a parallel pool in matlab before running get_clearances_octree_par.m but otherwise the process remains unchanged.

## Las Notes

The las file must have scan angle rank as a *standard* scalar field, scan angle rank as a extra data field or what have you will not work. Gpstime is also a required scalar field.

## Variables You May Consider Changing

### In The Variables Section

**target_plane_width**: controls the granularity of the measurements, smaller values require longer to process

**scanwidth**: controls how wide the scan area is

**maxheight**: the height at which the scan will stop and assume infinity

**maxside**: the distance on either side at which the scan will stop and assume infinity

**min_pts**: the number of points needed to consider something an obstruction, helps against noise

**candidate_buffer**: how much distance is needed between candidates to be considered separate candidates

**candidate_padding**: how much distance should be added to the sides of the contour plots

**sample_percent**: the percentage of points to *keep*. Only works divisions of 1 by multiples of 2. (1, 0.5, 0.25, etc.)

**translate_pts**: whether or not to translate the points closer to the origin. May or may not improve precision of results.

### In The Initial Plot Section

**side_clearance_plot_height**: at what height the data for the line graphs will be taken from

### In The Contour Plot Section

**toppad**: how much padding to have between the top of the candidate points and the top of the plot

**side_clearance_plot_height**: at what height the data for the line graphs will be taken from

## Some issues

Some combinations of observer_height and maxheight cause issues with the giftwrap algorithm for some reason, 3 and 15 respectively were observed to have issues.

Sections of interest have trailing empty space, this seems to be from the buffer system. Probably just need to trim the end of each section when the program determines it has ended.

## Explanatory Diagrams

Yes, these are my planning drawings with added labels. Yes, I can't draw and my writing is what it is.

![alt text](diagrams/20220225_151729.jpg?raw=true)
![alt text](diagrams/20220225_151741.jpg?raw=true)
