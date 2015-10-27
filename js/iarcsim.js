/******************\
| IARC Simulation  |
| @author Anthony  |
| @version 0.2.0.1 |
| @date 2015/10/24 |
| @edit 2015/10/26 |
\******************/

var IARCSim = (function() {
    'use strict';

    /**********
     * config */
    var DIMS = [500, 0];
        DIMS[1] = DIMS[0]; //force squareness
    var SPEEDUP = 2;

    var UAV_R = 0.0255*DIMS[0]; //arbitrary size of the uav
    var UAV_S = 0.15*DIMS[0]*SPEEDUP; //arbitrary speed of the uav

    var N = 10; //number of roombas
    var R = 0.0085*DIMS[0]; //radius of the Roombas
    var S = 0.0165*DIMS[0]*SPEEDUP; //speed in px per second
    var INIT_D = 0.05*500; //how far the roombas are initially
    var ANG_SPEED = 1.38*SPEEDUP; //angular speed in radians per second
    var FLIP_FREQ = 20*1000/SPEEDUP; //rotates 180 degrees every 20s
    var RAND_ANG_FREQ = 5*1000/SPEEDUP; //rotates randomly every 5s
    var DTHETA = 40*Math.PI/180; //how much it wiggles by for random rotations
    var ACT_ANGLE = 45*Math.PI/180; //how much it rotates upon activation

    var OBST_N = 4; //number of obstacle roombas
    var OBST_R = 1.5*R; //radius of the obstacle roombas
    var OBST_S = S; //speed of the obstacle roombas
    var OBST_INIT_D = 0.25*DIMS[0];
    var OBST_ANG_SPEED = 1.38*SPEEDUP; //angular speed in radians per second

    /****************
     * working vars */
    var canvas, ctx;
    var uav, roombas, obstacles;
    var globalTime;
    var lastRenderTime;
    var keys;

    /*************
     * constants */
    var CENTER = [DIMS[0]/2, DIMS[1]/2];

    /***********
     * objects */
    function UAV(pos, color) {
      this.position = pos.slice(0);
      this.color = color;
      this.direc = [0, 0];
      this.r = UAV_R;
      this.s = UAV_S;
    }
    UAV.prototype.move = function(dt) {
      //get the direction from the pressed keys
      var dir = [0, 0];
      if (keys[65]) dir = [dir[0]-1, dir[1]];
      if (keys[68]) dir = [dir[0]+1, dir[1]];
      if (keys[87]) dir = [dir[0], dir[1]-1];
      if (keys[83]) dir = [dir[0], dir[1]+1];
      dir = norm(dir);
      if (dir[0] === 0 && dir[1] === 0) return;

      //nonzero direction/ all good.
      var ds = this.s * dt/1000;
      this.direc = dir.slice(0);
      this.position[0] += ds * this.direc[0];
      this.position[1] += ds * this.direc[1];
    };

    function Roomba(pos, color) {
      this.position = pos.slice(0);
      this.color = color;
      this.direc = norm([
        this.position[0] - CENTER[0],
        this.position[1] - CENTER[1]
      ]);
      this.r = R; //radius
      this.s = S; //speed

      this.t1 = 0; //time since last flip
      this.t2 = 0; //time since last wiggle
      this.angleLeftToMove = 0; //how much this UAV needs to rotate
    }
    Roomba.prototype.update = function(dt) {
      this.move(dt);
      this.t1 += dt;
      if (this.t1 > RAND_ANG_FREQ) {
        this.t1 = 0;
        this.randomizeAngle();
      }
      this.t2 += dt;
      if (this.t2 > FLIP_FREQ) {
        this.t2 = 0;
        this.flip();
      }
    };
    Roomba.prototype.rotateByAngle = function(theta) {
      var currAng = Math.atan2(this.direc[1], this.direc[0]);
      this.direc[0] = Math.cos(currAng + theta);
      this.direc[1] = Math.sin(currAng + theta);
    };
    Roomba.prototype.queueRotation = function(theta) {
      this.angleLeftToMove = (this.angleLeftToMove+theta)%(2*Math.PI);
    };
    Roomba.prototype.move = function(dt, forceTranslation) {
      if (this.angleLeftToMove === 0 || forceTranslation) {
        var ds = this.s * dt/1000;
        this.position[0] += ds * this.direc[0];
        this.position[1] += ds * this.direc[1];
      } else {
        var dtheta = ANG_SPEED * dt/1000;
        this.rotateByAngle(dtheta);
        this.angleLeftToMove -= dtheta;
        if (this.angleLeftToMove < 0) this.angleLeftToMove = 0;
      }
    };
    Roomba.prototype.flip = function() {
      this.queueRotation(Math.PI);
    };
    Roomba.prototype.randomizeAngle = function() {
      var thetaChange = -DTHETA/2 + DTHETA*Math.random();
      this.rotateByAngle(thetaChange);
    };
    Roomba.prototype.distTo = function(ent) {
      return mag([
        this.position[0] - ent.position[0],
        this.position[1] - ent.position[1]
      ]);
    };
    Roomba.prototype.collideWith = function(roomba) {
      this.flip();
      this.rotateByAngle(Math.PI);
      this.move(1000/60, true); //assume 60fps
      this.rotateByAngle(Math.PI); //undo the rotation

      roomba.flip();
      roomba.rotateByAngle(Math.PI);
      roomba.move(1000/60, true); //assume 60fps
      roomba.rotateByAngle(Math.PI); //undo the rotation
    };
    Roomba.prototype.activateMagnet = function() {
      this.queueRotation(ACT_ANGLE);
    };

    function ObstacleRoomba(pos, color) {
      this.position = pos.slice(0);
      this.color = color;
      this.direc = norm([
        -this.position[1] + CENTER[1],
        this.position[0] - CENTER[0]
      ]); //perpendicular to initial position vector, clockwise
      this.r = OBST_R; //radius
      this.s = OBST_S; //speed

      this.angleLeftToMove = 0; //angle to pretend to rotate for synchronicity
    }
    ObstacleRoomba.prototype.move = function(dt) {
      if (this.angleLeftToMove !== 0) {
        var shadowTheta = OBST_ANG_SPEED * dt/1000;
        this.angleLeftToMove -= shadowTheta;
        if (this.angleLeftToMove < 0) this.angleLeftToMove = 0;
      } else {
        var theta = Math.atan2(
          this.position[1] - CENTER[1],
          this.position[0] - CENTER[0]
        );
        var dtheta = (OBST_S/OBST_INIT_D) * dt/1000;
        this.direc = [
          -Math.sin(theta+dtheta),
          Math.cos(theta+dtheta)
        ];
        this.position = [
          OBST_INIT_D*this.direc[1] + CENTER[1],
          -OBST_INIT_D*this.direc[0] + CENTER[0]
        ];
      }
    };
    ObstacleRoomba.prototype.shadowRotation = function(theta) {
      this.angleLeftToMove += theta;
    };
    ObstacleRoomba.prototype.collideWith = function(roomba) {
      //remove the overlap
      var oHat = norm([
        this.position[0] - roomba.position[0],
        this.position[1] - roomba.position[1]
      ]);
      var dist = roomba.distTo(this);
      var oMag = (this.r + roomba.r - dist) + 2;
      roomba.position = [
        roomba.position[0] - oMag*oHat[0],
        roomba.position[1] - oMag*oHat[1]
      ];

      //rotate the roomba away
      roomba.flip();

      //shadow the rotation of the roomba for synchronicity
      this.shadowRotation(1.1*Math.PI); //for timing purposes
    };

    /******************
     * work functions */
    function initIARCSim() {
      canvas = $s('#canvas');
      canvas.width = DIMS[0];
      canvas.height = DIMS[1];
      ctx = canvas.getContext('2d');

      //uav initialization
      uav = new UAV(
        CENTER,
        Crush.getColorStr([
          Math.floor(80*Math.random()),
          Math.floor(80*Math.random()),
          Math.floor(80*Math.random())
        ], 0.3)
      );

      //roomba initialization
      roombas = [];
      for (var ai = 0; ai < N; ai++) {
        var theta = ai*(2*Math.PI/N);
        var x = INIT_D*Math.cos(theta);
        var y = INIT_D*Math.sin(theta);
        roombas.push(
          new Roomba(
            [CENTER[0]+x, CENTER[1]+y],
            Crush.getColorStr([
              Math.floor(140*Math.random()),
              Math.floor(140*Math.random()),
              Math.floor(140*Math.random())
            ])
          )
        );
      }

      //obstacle init
      obstacles = [];
      for (var ai = 0; ai < OBST_N; ai++) {
        var theta = ai*(2*Math.PI/OBST_N);
        var x = OBST_INIT_D*Math.cos(theta);
        var y = OBST_INIT_D*Math.sin(theta);
        obstacles.push(
          new ObstacleRoomba(
            [CENTER[0]+x, CENTER[1]+y],
            'red'
          )
        );
      }

      //init time stuff
      globalTime = 0;
      lastRenderTime = +new Date();
      $s('#t').innerHTML = fmtTime(0, 0);
      $s('#num-robots').innerHTML = roombas.length;

      //controls
      keys = [];
      window.addEventListener('keydown', function(e) {
        if (e.keyCode === 32) return;
        keys[e.keyCode] = true;
      });
      window.addEventListener('keyup', function(e) {
        keys[e.keyCode] = false;
        if (e.keyCode === 32) keys[e.keyCode] = true; //reversed
      });

      render();
    }

    function handleRoombaCollisions() {
      for (var ai = 0; ai < roombas.length; ai++) {
        for (var bi = ai+1; bi < roombas.length; bi++) {
          //collisions
          if (roombas[ai].distTo(roombas[bi]) < roombas[ai].r+roombas[bi].r) {
            roombas[ai].collideWith(roombas[bi]);
          }
        }
      }
    }

    function handleObstacleCollisions() {
      for (var ai = 0; ai < obstacles.length; ai++) {
        var obstacle = obstacles[ai];
        for (var bi = 0; bi < roombas.length; bi++) {
          //collisions
          if (roombas[bi].distTo(obstacle) < roombas[bi].r + obstacle.r) {
            obstacle.collideWith(roombas[bi]);
          }
        }
      }
    }

    function handleExitBehavior() {
      for (var ai = 0; ai < roombas.length; ai++) {
        if (roombas[ai].position[0] < roombas[ai].r ||
            roombas[ai].position[0] > DIMS[0]-roombas[ai].r ||
            roombas[ai].position[1] < roombas[ai].r ||
            roombas[ai].position[1] > DIMS[1]-roombas[ai].r) {
          roombas.splice(ai, 1);
          $s('#num-robots').innerHTML = roombas.length;
          ai--; //it'll get incremented and ai will remain the same
        }
      }
    }

    function handleMagnetActivation() {
      if (keys[32]) {
        //prevent this from happening many times during a key-hold
        keys[32] = false;

        //get the closest roomba
        var closestRoomba = roombas.reduce(
          function(closest, curr, idx) {
            var dist = mag([
              curr.position[0] - uav.position[0],
              curr.position[1] - uav.position[1]
            ]);
            if (dist < closest[1]) return [idx, dist];
            else return closest;
          },
          [-1, Infinity] //idx of closest, distance
        );

        //make sure the closest is close enough
        if (closestRoomba[1] < Math.abs(uav.r - roombas[closestRoomba[0]].r)) {
          //if it is, rotate it
          roombas[closestRoomba[0]].activateMagnet();
        }
      }
    }

    function render() {
      clearCanvas();

      //deal with time
      var t = +new Date();
      var dt = t - lastRenderTime;
      lastRenderTime = t;
      globalTime += dt*SPEEDUP;
      $s('#t').innerHTML = fmtTime(globalTime);

      //draw the roomba positions
      roombas.map(drawEntity);

      //draw the obstacles
      obstacles.map(drawEntity);

      //render the UAV
      drawEntity(uav);

      //roomba-roomba collisions
      handleRoombaCollisions();

      //roomba-obstacle collisions
      handleObstacleCollisions();

      //roomba velocity updates
      roombas.map(function(roomba) {
        roomba.update(dt);
      });

      //obstacle velocity updates
      obstacles.map(function(obstacle) {
        obstacle.move(dt);
      });

      //remove the roombas that've crossed the wall
      handleExitBehavior();

      //move the UAV
      uav.move(dt);

      //uav-roomba interaction
      handleMagnetActivation();

      requestAnimationFrame(render);
    }

    /********************
     * helper functions */
    function fmtTime(t) { //t is in ms
      var tis = t/1000;
      var mins = Math.floor(tis/60);
      var secs = Math.round(tis - mins*60);
      return mins+'m '+secs+'s';
    }

    function mag(a) {
      return Math.sqrt(a.reduce(function(acc, curr) {
        return acc + curr*curr;
      }, 0));
    }

    function norm(a) {
      var magnitude = mag(a);
      if (magnitude === 0) return a;
      return a.map(function(comp) {
        return comp/magnitude;
      });
    }

    function drawEntity(ent) {
      //represent the Roomba
      Crush.drawPoint(ctx, ent.position, ent.r, ent.color);
      //velocity
      Crush.drawLine(ctx, ent.position, [
        ent.position[0] + 2*ent.r*ent.direc[0],
        ent.position[1] + 2*ent.r*ent.direc[1]
      ], ent.color, 2);
    }

    function clearCanvas() {
      ctx.fillStyle = 'white';
      ctx.fillRect(0, 0, canvas.width, canvas.height);
    }

    function $s(id) { //for convenience
      if (id.charAt(0) !== '#') return false;
      return document.getElementById(id.substring(1));
    }

    return {
      init: initIARCSim,
      render: render,
      obstacles: obstacles
    };
})();

window.addEventListener('load', IARCSim.init);
