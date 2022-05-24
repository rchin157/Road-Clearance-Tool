function constraints = get_constraint(obsvr, trgt)
hull_points = [obsvr; trgt(1,:); trgt(2,:); trgt(3,:); trgt(4,:)];
convex_hull = giftwrap3d_mex(hull_points);
constraints = convert_hull_to_constraints(convex_hull, hull_points, 0);
