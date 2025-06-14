1. combine tof data
    <br> plot:
    <ul> 
        <li> keep all colors constant across all plots. Subplots?
        <li> z vs. joint angle, linear
        <li> z vs. xy, 2D
        <li> z vs x vs y, 3D projection
    </ul>
1. group combined tof data into two curves, if possible
1. run curve fitting
    <ul>
        <li> refactor to work without sensor names.
        <li> maintain original joint angles, timestamps
        <li> does this still need to find a ts min? Do I need to ask for tf frames? Should I just add a MoveIt joint pose request to my CPP controller? 
        <ul> 
            <li> Let's say that the branch is above the center of rotation
            <li> the tof overlaps will not read at symmetrical joint angles.
            <li> If I find wrist joint angles, I will need to do forward kinematics? 
            <li> Might as well ask for the tf frame, but how am I to sequentially get the tf frame for a single sensor?
        </ul>

    </ul>

