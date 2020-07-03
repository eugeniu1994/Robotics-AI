world {}
table (world){
    shape:ssBox, Q:<t(0 0. .01)>, size:[2. 1.1, .1 .02], color:[.3 .3 .3], mass:0.5,
    contact, logical:{ }
    friction:.5
}

#walls------------------------------------------------
left_wall (world){
    shape:ssBox, Q:<t(0 -.55 .05)>, size:[2. .05 .05 .02], color=[0 .3 .7]
    contact, logical:{ }
    friction:.001
}

right_wall (world){
    shape:ssBox, Q:<t(0 .55 .05)>, size:[2. .05 .05 .02], color=[0 .3 .7]
    contact, logical:{ }
    friction:.001
}

#tools-------------------------------------------------
#Right========
tool_right (table){
    shape:ssBox, Q:<t(0.85 -0.5 .15) d(90 0 0 1) >, size:[.005 .2 .2 0.001], color:[.2 .9 .2]
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
    shape:ssBox, Q:<t(-0.85 .5 .15) d(90 0 0 1)>, size:[.005 .2 .2 0.001], color:[.2 .9 .2]
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
        
Edit L_panda_link0 (table)  { Q:<t(-0.9 0 .05) d(0 0 0 0)>}
Edit R_panda_link0 (table)  { Q:<t( 0.9 0 .05) d(180 0 0 1)>}

#Target--------------------------------------------------
target 	{  shape:sphere, size:[.1], color=[1. 0. 0.],friction:.005, mass:0.015, X:<[.45, .2, .2,  0.511492, 0.409407, -0.742116, -0.141515]> }


#Camera--------------------------------------------------
camera(world){
    Q:<t(0 0 1.3) d(180 0 0 1)>,
    shape:ssBox, color=[0. 0. 1.], size:[.02 .02 .02 .02],
    focalLength:0.895, width:640, height:360, zRange:[.5 100]
}

#----------------------------------------------------------------------------------------------------------------------------------
#Referee here-

#body world {  }
body (world){}
body base_footprint { mass=140.967  }

shape base_link_1 (base_footprint){ type=mesh rel=<T -0.0282159 0.00517696 0.272424 1 0 0 0 >  mesh='base_v0/base.stl'  meshscale=0.06  rel_includes_mesh_center,  }
shape base_footprint_1 (base_footprint){ type=box  size=[ 0.01 0.01 0.01 0 ]  }
shape fl_caster_rotation_link_1 (base_footprint){ type=mesh rel=<T 0.21838 0.225407 0.180919 1 0 0 0 >  mesh='base_v0/caster.stl'  rel_includes_mesh_center,  }
shape fl_caster_l_wheel_link_1 (base_footprint){ type=mesh rel=<T 0.224225 0.270791 0.0789213 1 0 0 0 >  mesh='base_v0/wheel.stl'  rel_includes_mesh_center,  }
shape fl_caster_r_wheel_link_1 (base_footprint){ type=mesh rel=<T 0.224225 0.172791 0.0789213 1 0 0 0 >  mesh='base_v0/wheel.stl'  rel_includes_mesh_center,  }
shape fr_caster_rotation_link_1 (base_footprint){ type=mesh rel=<T 0.21838 -0.223793 0.180919 1 0 0 0 >  mesh='base_v0/caster.stl'  rel_includes_mesh_center,  }
shape fr_caster_l_wheel_link_1 (base_footprint){ type=mesh rel=<T 0.224225 -0.178409 0.0789213 1 0 0 0 >  mesh='base_v0/wheel.stl'  rel_includes_mesh_center,  }
shape fr_caster_r_wheel_link_1 (base_footprint){ type=mesh rel=<T 0.224225 -0.276409 0.0789213 1 0 0 0 >  mesh='base_v0/wheel.stl'  rel_includes_mesh_center,  }
shape bl_caster_rotation_link_1 (base_footprint){ type=mesh rel=<T -0.23082 0.225407 0.180919 1 0 0 0 >  mesh='base_v0/caster.stl'  rel_includes_mesh_center,  }
shape bl_caster_l_wheel_link_1 (base_footprint){ type=mesh rel=<T -0.224975 0.270791 0.0789213 1 0 0 0 >  mesh='base_v0/wheel.stl'  rel_includes_mesh_center,  }
shape bl_caster_r_wheel_link_1 (base_footprint){ type=mesh rel=<T -0.224975 0.172791 0.0789213 1 0 0 0 >  mesh='base_v0/wheel.stl'  rel_includes_mesh_center,  }
shape br_caster_rotation_link_1 (base_footprint){ type=mesh rel=<T -0.23082 -0.223793 0.180919 1 0 0 0 >  mesh='base_v0/caster.stl'  rel_includes_mesh_center,  }
shape br_caster_l_wheel_link_1 (base_footprint){ type=mesh rel=<T -0.224975 -0.178409 0.0789213 1 0 0 0 >  mesh='base_v0/wheel.stl'  rel_includes_mesh_center,  }
shape br_caster_r_wheel_link_1 (base_footprint){ type=mesh rel=<T -0.224975 -0.276409 0.0789213 1 0 0 0 >  mesh='base_v0/wheel.stl'  rel_includes_mesh_center,  }

joint worldTranslationRotation (world base_footprint){ type=transXYPhi  ctrl_H=20  }

## BASE
Edit worldTranslationRotation { Q:< t(0 -1.5 0.1) d(90 0 0 1)> gains=[1 1] ctrl_limits=[1 1 1] ctrl_H=5 base }

Prefix: "Ref_"
Include: 'panda_moveGripper.g'
Prefix!
Edit Ref_panda_link0 (base_footprint)  { Q:<t(0.15 0 0.) d(0 0 0 0)>}

#walls 2------------------------------------------------
left_wall2 (world){
    shape:ssBox, Q:<t(0 -2.65 .05)>, size:[7. .02 .2  .02], color=[0 1. 1.]
    contact, logical:{ }
    friction:.5
}

left_wall2_1 (world){
    shape:ssBox, Q:<t(-3.5 0 .05)>, size:[.02 5.3 .2  .02], color=[0 1. 1.]
    contact, logical:{ }
    friction:.5
}

right_wall2 (world){
    shape:ssBox, Q:<t(0 2.65 .05)>, size:[7. .02 .2  .02], color=[0 1. 1.]
    contact, logical:{ }
    friction:.5
}

left_wall2_2 (world){
    shape:ssBox, Q:<t(3.5 0 .05)>, size:[.02 5.3 .2 .02], color=[0 1. 1.]
    contact, logical:{ }
    friction:.5
}

