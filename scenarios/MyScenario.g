world {}
table (world){
    shape:ssBox, Q:<t(0 0. .01)>, size:[2. 1.2, .1 .02], color:[.3 .3 .3]
    contact, logical:{ }
    friction:.5
}

#walls------------------------------------------------
left_wall (world){
    shape:ssBox, Q:<t(0 -.6 .05)>, size:[2. .02 .05 .02], color=[0 .3 .7]
    contact, logical:{ }
    friction:.001
}

right_wall (world){
    shape:ssBox, Q:<t(0 .6 .05)>, size:[2. .02 .05 .02], color=[0 .3 .7]
    contact, logical:{ }
    friction:.001
}

#tools-------------------------------------------------
#Right========
tool_right (table){
    shape:ssBox, Q:<t(0.85 -0.5 .15) d(90 0 0 1) >, size:[.02 .2 .2 .001], color:[.2 .9 .2]
    joint:rigid
    friction:.1
}
ring_ (tool_right){
    Q:<t(0 0 .2)>
}
ring_1(ring_){ shape:capsule, Q:<t(0 0 -.1)>, size:[.05 .01], color:[.6 .6 .6], logical: {object} }
ring_2(ring_){ shape:ssBox, Q:<t(0 0 -.05) d(90 0 1 0)>, size:[.05 .05 .001 .01], color:[.2 .9 .2], logical: {object} }

#Left========
tool_left (table){
    shape:ssBox, Q:<t(-0.85 .5 .15) d(90 0 0 1)>, size:[.02 .2 .2 .001], color:[.2 .9 .2]
    joint:rigid
    friction:.1
}
ring (tool_left){
    Q:<t(0 0 .2)>
}
ring1(ring){ shape:capsule, Q:<t(0 0 -.1)>, size:[.05 .01], color:[.6 .6 .6], logical: {object} }
ring2(ring){ shape:ssBox, Q:<t(0 0 -.05) d(90 0 1 0)>, size:[.05 .05 .001 .01], color:[.2 .9 .2], logical: {object} }

#Robots -------------------------------------------------
Prefix: "L_"
Include: 'panda_moveGripper.g'

Prefix: "R_"
Include: 'panda_moveGripper.g'

Prefix!
        
Edit L_panda_link0 (table)  { Q:<t(-0.85 0 .05) d(0 0 0 0)>}
Edit R_panda_link0 (table)  { Q:<t( 0.85 0 .05) d(180 0 0 1)>}

#Target--------------------------------------------------
target 	{  shape:sphere, size:[.1], color=[1. 0. 0.], mass:0.02 X:<[.35, .2, .2,  0.511492, 0.409407, -0.742116, -0.141515]> }

#Camera--------------------------------------------------
camera(world){
    Q:<t(-0.01 -.2 1.8) d(30 1 0 0)>,
    shape:ssBox, color=[0. 0. 1.], size:[.02 .02 .02 .02],
    focalLength:0.895, width:640, height:360, zRange:[.5 100]
}
