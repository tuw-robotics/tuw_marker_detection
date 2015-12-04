# This expresses a ellipse equation


#id of the ellipse
uint32 id


# ellipse equation  p' C p = 0;
#     [ A  B  D]
# C = [ B  C  E]
#     [ D  E  F]
#
# p = [x, y, 1]' --> p is a image point
float64[9] C



# The following data holds redundent information
# locaiton of the ellipse 
geometry_msgs/Pose2D[] Pose2D
# radius of the ellipse radius[0] > radius[1]
float64[2] radius

 