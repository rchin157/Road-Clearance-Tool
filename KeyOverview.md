# Overview of Key Sections (Methodology)

## Brief recap of methods used in previous works

This project uses an octree for organizing and querying the pointcloud. In brief, closed 3d shapes represented by halfspace equations are used to query the octree for points contained within such shapes. As discussed in some of our other projects the use of convex hulls and octrees provides benefits over other methods such as raycasting. The trajectory of the scan vehicle is obtained from the set of points with a scan angle rank of 0, that is directly below the vehicle, which are first formed into a smoothed path and then interpolated to yield a regularly spaced set of road points as the trajectory. At the same time directional vectors at each road point are obtained, these include forwards, leftwards, and frontwards vectors.

## Determining target planes

Target planes (or scan targets) are determined from their associated observer along side the matching leftwards and forwards vector determined earlier. These targets are necessary to form the frustum shapes used for querying. To emulate a "scanline" a row of these targets is required to collect data along such a line. As such, the row of observers is generated from an original road point and an array of integer offsets. It should be noted that internally scan targets are bigger than the target_plane_width variable due to the nature of the frustum shape used. Naturally such a shape is similar to a pyramid and as a result has a smaller cross section closer to the observer point. Since ideally the end of the frustum extends well beyond what is intended to be scanned, this may result in points being missed by a query due to the smaller cross section at the desired scan location. The increase in size should serve to mitigate this effect.

There are two types of targets, vertical and horizontal, where the vertical ones scan towards the sides of the road and the horizontal ones scan towards the sky. Each type has its own offsets and observer points to complete their respective roles.

## Measuring clearances

Clearance results are stored as three n by m matrices where n is the number of scantiles, and m is the number of road points. There is one matrix for top, left, and right clearance data. For every road point and each associated scantile, half space constraints are determined from the observer and target plane points generated as mentioned above. With these constraints an octree is queried for points within the defined frustum. To reduce the influence of noise and scan artifacts, a minimum number of points must be returned in order for the them to be considered, failing to meet this requirement yields a clearance of the maximum allowable range.

### Top Clearance

Of the points returned, if any, the top clearance for the current observer and scantile combo is computed as the absolute difference in z coordinates between the observer point with the observer height subtracted and the lowest point (z-wise) returned from the query.

### Side Clearance

Points are similarly queried here as for top clearance. From there the euclidean distance between the returned points and the current observer point is computed. The side clearance is then taken to be the lowest distance found.

### Results and Visualization

The arrays of scanning targets serve to capture data at multiple points so that when combined with data from other road points, a 2D depth map is formed which enables an intuitive visualization in the form of contour maps. The side clearance maps have been formatted such that they appear as if the viewer was seated within the scan vehicle and looked out the window and the top clearance map as if looking up through the roof of the vehicle.

Aside from the contour maps results are also generated in the form of simple line plots representing clearance directly above the vehicle along the trajectory as well as at a single fixed height along both sides which shows the major clearance profile at a glance.

## Filtering Sections of Interest

At present the filtering algorithm is exceedingly simple and probably not terribly robust against small overhanging objects and other such conditions that were unavailable during testing. It does however perform quite well on open roads with overpasses and such. Simply put, a section of interest is any road points where the top clearance is lower than the average top clearance value. From there it simply records all the nearby road points into a cell with each section being a new cell. The way it determines what should be a contiguous section of interest is a combination of the recording of nearby road points as mentioned, and a buffer system. The buffer system forces the algorithm to keep recording points even if the current top clearance is higher than average, and it must encounter a certain number of sequential points that are all higher than average before it gives up and considers the current section complete.