function sv = steering_Vector(lambda)

    antena_distance = lambda/2 ;
    oneside_angle = 90;
    degree = 1;
    pos = 0:antena_distance:antena_distance*3;
    ang = -oneside_angle:degree:oneside_angle;
    sv = steervec(pos,ang,0);

end