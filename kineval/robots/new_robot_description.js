robot = {
    name:"h1", 
    partner:"jessisc",
    base:"pelvis",
    origin: {xyz: [0, 1.05, 0], rpy: [0,0,0]},
    links: {
        "pelvis": {
            visual: {
                origin: { xyz: [0, 0, 0], rpy: [0,0,0]},
                geometry: {mesh: {filename:"meshes/pelvis.dae"}},
            }
        },

        "left_hip_yaw_link": {
            visual: {
                origin: { xyz: [0, 0, 0], rpy: [0,0,0]},
                geometry: {mesh: {filename:"meshes/left_hip_yaw_link.dae"}},
            }
        },

        "left_hip_roll_link": {
            visual: {
                origin: { xyz: [0, 0, 0], rpy: [0,0,0]},
                geometry: {mesh: {filename:"meshes/left_hip_roll_link.dae"}},
            }
        },

        "left_hip_pitch_link": {
            visual: {
                origin: { xyz: [0, 0, 0], rpy: [0,0,0]},
                geometry: {mesh: {filename:"meshes/left_hip_pitch_link.dae"}},
            }
        },

        "left_knee_link": {
            visual: {
                origin: { xyz: [0, 0, 0], rpy: [0,0,0]},
                geometry: {mesh: {filename:"meshes/left_knee_link.dae"}},
            }
        },

        "left_ankle_link": {
            visual: {
                origin: { xyz: [0, 0, 0], rpy: [0,0,0]},
                geometry: {mesh: {filename:"meshes/left_ankle_link.dae"}},
            }
        },

        "right_hip_yaw_link": {
            visual: {
                origin: { xyz: [0, 0, 0], rpy: [0,0,0]},
                geometry: {mesh: {filename:"meshes/right_hip_yaw_link.dae"}},
            }
        },

        "right_hip_roll_link": {
            visual: {
                origin: { xyz: [0, 0, 0], rpy: [0,0,0]},
                geometry: {mesh: {filename:"meshes/right_hip_roll_link.dae"}},
            }
        },

        "right_hip_pitch_link": {
            visual: {
                origin: { xyz: [0, 0, 0], rpy: [0,0,0]},
                geometry: {mesh: {filename:"meshes/right_hip_pitch_link.dae"}},
            }
        },

        "right_knee_link": {
            visual: {
                origin: { xyz: [0, 0, 0], rpy: [0,0,0]},
                geometry: {mesh: {filename:"meshes/right_knee_link.dae"}},
            }
        },

        "right_ankle_link": {
            visual: {
                origin: { xyz: [0, 0, 0], rpy: [0,0,0]},
                geometry: {mesh: {filename:"meshes/right_ankle_link.dae"}},
            }
        },

        "torso_link": {
            visual: {
                origin: { xyz: [0, 0, 0], rpy: [0,0,0]},
                geometry: {mesh: {filename:"meshes/torso_link.dae"}},
            }
        },

        "left_shoulder_pitch_link": {
            visual: {
                origin: { xyz: [0, 0, 0], rpy: [0,0,0]},
                geometry: {mesh: {filename:"meshes/left_shoulder_pitch_link.dae"}},
            }
        },

        "left_shoulder_roll_link": {
            visual: {
                origin: { xyz: [0, 0, 0], rpy: [0,0,0]},
                geometry: {mesh: {filename:"meshes/left_shoulder_roll_link.dae"}},
            }
        },

        "left_shoulder_yaw_link": {
            visual: {
                origin: { xyz: [0, 0, 0], rpy: [0,0,0]},
                geometry: {mesh: {filename:"meshes/left_shoulder_yaw_link.dae"}},
            }
        },

        "left_elbow_link": {
            visual: {
                origin: { xyz: [0, 0, 0], rpy: [0,0,0]},
                geometry: {mesh: {filename:"meshes/left_elbow_link.dae"}},
            }
        },
        
        "right_shoulder_pitch_link": {
            visual: {
                origin: { xyz: [0, 0, 0], rpy: [0,0,0]},
                geometry: {mesh: {filename:"meshes/right_shoulder_pitch_link.dae"}},
            }
        },

        "right_shoulder_roll_link": {
            visual: {
                origin: { xyz: [0, 0, 0], rpy: [0,0,0]},
                geometry: {mesh: {filename:"meshes/right_shoulder_roll_link.dae"}},
            }
        },

        "right_shoulder_yaw_link": {
            visual: {
                origin: { xyz: [0, 0, 0], rpy: [0,0,0]},
                geometry: {mesh: {filename:"meshes/right_shoulder_yaw_link.dae"}},
            }
        },

        "right_elbow_link": {
            visual: {
                origin: { xyz: [0, 0, 0], rpy: [0,0,0]},
                geometry: {mesh: {filename:"meshes/right_elbow_link.dae"}},
            }
        },
    },
 }
  // end links


// specify name of endeffector frame
robot.endeffector = {};
robot.endeffector.frame = "left_hip_yaw_joint";
robot.endeffector.position = [[1],[1],[1],[1]]

robot.joints = {};


robot.joints.left_hip_yaw_joint = { 
    type : "revolute",
    parent: "pelvis", child: "left_hip_yaw_link",
    axis : [0,0,1],
    origin : {xyz: [0, 0.0875, -0.1742], rpy:[0,0,0]},
    limit : {lower:-0.43, upper:0.43}
};
robot.joints.left_hip_roll_joint = { 
    type : "revolute",
    parent: "left_hip_yaw_link", child: "left_hip_roll_link",
    axis : [1,0,0],
    origin : {xyz: [0.039468, 0, 0], rpy:[0,0,0]},
    limit : {lower:-0.43, upper:0.43}
};

robot.joints.left_hip_pitch_joint = { 
    type : "revolute",
    parent: "left_hip_roll_link", child: "left_hip_pitch_link",
    axis : [0,1,0],
    origin : {xyz: [0, 0.11536, 0], rpy:[0,0,0]},
    limit : {lower:-3.14, upper:2.53}
};

robot.joints.left_knee_joint= { 
    type : "revolute",
    parent: "left_hip_pitch_link", child: "left_knee_link",
    axis : [0,1,0],
    origin : {xyz: [0,0,-0.4], rpy:[0,0,0]},
    limit : {lower:-0.26, upper:2.05}
};

robot.joints.left_ankle_joint= { 
    type : "revolute",
    parent: "left_knee_link", child: "left_ankle_link",
    axis : [0,0,1],
    origin : {xyz: [0,0,-0.4], rpy:[0,0,0]},
    limit : {lower:-0.87, upper:0.52}
};

robot.joints.right_hip_yaw_joint= { 
    type : "revolute",
    parent: "pelvis", child: "right_hip_yaw_link",
    axis : [0,1,0],
    origin : {xyz: [0, -0.0875, -0.1742], rpy:[0,0,0]},
    limit : {lower:-0.43, upper:0.43}
};

robot.joints.right_hip_roll_joint = { 
    type : "revolute",
    parent: "right_hip_yaw_link", child: "right_hip_roll_link",
    axis : [1,0,0],
    origin : {xyz: [0.039468, 0, 0], rpy:[0,0,0]},
    limit : {lower:-0.43, upper:0.43}
};


robot.joints.right_hip_pitch_joint = { 
    type : "revolute",
    parent: "right_hip_roll_link", child: "right_hip_pitch_link",
    axis : [0,1,0],
    origin : {xyz: [0, -0.11536, 0], rpy:[0,0,0]},
    limit : {lower:-3.14, upper:2.53}
};


robot.joints.right_knee_joint= { 
    type : "revolute",
    parent: "right_hip_pitch_link", child: "right_knee_link",
    axis : [0,1,0],
    origin : {xyz: [0,0,-0.4], rpy:[0,0,0]},
    limit : {lower:-0.26, upper:2.05}
};


robot.joints.right_ankle_joint= { 
    type : "revolute",
    parent: "right_knee_link", child: "right_ankle_link",
    axis : [0,1,0],
    origin : {xyz: [0,0,-0.4], rpy:[0,0,0]},
    limit : {lower:-0.87, upper:0.52}
};

robot.joints.torso_joint= { 
    type : "revolute",
    parent: "pelvis", child: "torso_link",
    axis : [0,0,1],
    origin : {xyz: [0,0,0], rpy:[0,0,0]},
    limit : {lower:-2.35, upper:2.35}
};

robot.joints.left_shoulder_pitch_joint= { 
    type : "revolute",
    parent: "torso_link", child: "left_shoulder_pitch_link",
    axis : [0,1,0],
    origin : {xyz: [0.0055, 0.15535, 0.429990], rpy:[0.43633,0,0]},
    limit : {lower:-2.87, upper:2.87}
};

robot.joints.left_shoulder_roll_joint= { 
    type : "revolute",
    parent: "left_shoulder_pitch_link", child: "left_shoulder_roll_link",
    axis : [1,0,0],
    origin : {xyz: [-0.0055, 0.0565, -0.0165], rpy:[-0.43633,0,0]},
    limit : {lower:-0.34, upper:3.11}
};

robot.joints.left_shoulder_yaw_joint= { 
    type : "revolute",
    parent: "left_shoulder_roll_link", child: "left_shoulder_yaw_link",
    axis : [0,0,1],
    origin : {xyz: [0, 0, -0.1343], rpy:[0,0,0]},
    limit : {lower:-1.3, upper:4.45}
};

robot.joints.left_elbow_joint= { 
    type : "revolute",
    parent: "left_shoulder_yaw_link", child: "left_elbow_link",
    axis : [0,1,0],
    origin : {xyz: [0.0185, 0, -0.198], rpy:[0,0,0]},
    limit : {lower:-1.25, upper:2.61}
};


robot.joints.right_shoulder_pitch_joint= { 
    type : "revolute",
    parent: "torso_link", child: "right_shoulder_pitch_link",
    axis : [0,1,0],
    origin : {xyz: [0.0055, -0.15535, 0.42999], rpy:[-0.43633,0,0]},
    limit : {lower:-2.87, upper:2.87}
};


robot.joints.right_shoulder_roll_joint= { 
    type : "revolute",
    parent: "right_shoulder_pitch_link", child: "right_shoulder_roll_link",
    axis : [1,0,0],
    origin : {xyz: [-0.0055, -0.0565, -0.0165], rpy:[0.43633,0,0]},
    limit : {lower:-3.11, upper:0.34}
};

robot.joints.right_shoulder_yaw_joint= { 
    type : "revolute",
    parent: "right_shoulder_roll_link", child: "right_shoulder_yaw_link",
    axis : [0,0,1],
    origin : {xyz: [0, 0, -0.1343], rpy:[0,0,0]},
    limit : {lower:-4.45, upper:1.3}
};

robot.joints.right_elbow_joint= { 
    type : "revolute",
    parent: "right_shoulder_yaw_link", child: "right_elbow_link",
    axis : [0,1,0],
    origin : {xyz: [0.0185, 0, -0.198], rpy:[0,0,0]},
    limit : {lower:-1.25, upper:2.61}
};



// note ROS coordinate system (x:forward, y:lateral, z:up) is different than threejs (x:lateral, y:up, z:forward)
robot.links_geom_imported = true;

links_geom = {};

  // KE: replace hardcoded robot directory
  // KE: replace file extension processing
i = 0;
for (x in robot.links) {
  //geom_index = robot.links[x].visual.geometry.mesh.filename.split('_adjusted')[0];
  //geom_extension = robot.links[x].visual.geometry.mesh.filename.split('_adjusted')[1];
  filename_split = robot.links[x].visual.geometry.mesh.filename.split('.');
  geom_index = filename_split[0];
  geom_extension = filename_split[filename_split.length-1];
  console.log(geom_index + "  " + geom_extension);
  //assignFetchModel('./robots/sawyer/'+robot.links[x].visual.geometry.mesh.filename,geom_index);
  if (geom_extension === "dae") { // extend to use regex
    assignFetchModelCollada('./robots/'+robot.links[x].visual.geometry.mesh.filename,x);
  }
  else if (geom_extension === "DAE") { // extend to use regex
    assignFetchModelCollada('./robots/'+robot.links[x].visual.geometry.mesh.filename,x);
  }
  else {
    assignFetchModelSTL('./robots/'+robot.links[x].visual.geometry.mesh.filename,robot.links[x].visual.material,x);
  }
  i++;
}

function assignFetchModelCollada(filename,index) {

    console.log("assignFetchModel : "+filename+" - "+index); 
    var collada_loader = new THREE.ColladaLoader();
    var val = collada_loader.load(filename, 
       function ( collada ) {
            links_geom[index] = collada.scene;
        },
        function (xhr) {
            console.log(filename+" - "+index+": "+(xhr.loaded / xhr.total * 100) + '% loaded' );
        }
    );
}

function assignFetchModelCollada2(filename,index) {

    console.log("assignFetchModel : "+filename+" - "+index); 
    var collada_loader = new ColladaLoader2();
    var val = collada_loader.load(filename, 
       function ( collada ) {
            links_geom[index] = collada.scene;
        },
        function (xhr) {
            console.log(filename+" - "+index+": "+(xhr.loaded / xhr.total * 100) + '% loaded' );
        }
    );
}


function assignFetchModelSTL(filename,material_urdf,linkname) {

    console.log("assignFetchModel : "+filename+" - "+linkname); 
    var stl_loader = new THREE.STLLoader();
    var val = stl_loader.load(filename, 
       function ( geometry ) {
            // ocj: add transparency
            var material_color = new THREE.Color(material_urdf.color.rgba[0], material_urdf.color.rgba[1], material_urdf.color.rgba[2]);
            var material = new THREE.MeshLambertmaterial( {color: material_color, side: THREE.DoubleSide} );
            links_geom[linkname] = new THREE.Mesh( geometry, material ) ;
        } //,
        //function (xhr) {
        //    console.log(filename+" - "+linkname+": "+(xhr.loaded / xhr.total * 100) + '% loaded' );
        //}
    );
}




