
world {}

table (world){
    shape:ssBox, Q:<t(0 0. .01)>, size:[50. 50. .1 .02], color:[.3 .3 .3]
    contact, logical:{ }
    friction:.1
}

body world { X=<T t(0 0 .1)> type=0  size=[.01 .01 .01 0] fixed }
body transx { type=0  size=[.01 .01 .01 0] }
body transxy { type=0 size=[.01 .01 .01 0] }
body car {type=0  size=[0.2 0.5 .1 .0] color=[.8 0 0] }

joint (world transx) { type=0 }
joint (transx transxy) { type=0 }
joint (transxy car) { A=<T d(-90 0 1 0)> B=<T d(90 0 1 0)> type=0 }

camera(world){
    Q:<t(-0.01 -.2 1.8) d(30 1 0 0)>,
    shape:marker, size:[.1],
    focalLength:0.895, width:640, height:360, zRange:[.5 100]
}
