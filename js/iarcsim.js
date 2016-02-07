/********************\
| IARC Simulation AI |
| @author Anthony    |
| @version 1.0       |
| @date 2015/10/24   |
| @edit 2016/01/24   |
\********************/

var IARCSim = (function() {
    'use strict';

    /**********
     * config */
    var DIMS = [500, 0];
        DIMS[1] = DIMS[0]; //force squareness
    var NUM_GRID_LINES = [20, 20]; //how many grid lines to draw in each dir
    var SPEEDUP = 2.3;
    var MAX_TIME = 10*60*1000; //in ms

    var UAV_SPEC = {
      R: 0.0255*DIMS[0], //arbitrary size of the uav
      S: 0.15*DIMS[0]*SPEEDUP //arbitrary speed of the uav
    };
    var ROOMBA_SPEC = {
      N: 10, //number of roombas
      R: 0.0085*DIMS[0], //radius of the Roombas
      S: 0.0165*DIMS[0]*SPEEDUP, //speed in px per second
      initD: 0.05*500, //how far the roombas are initially
      angSpd: 1.38*SPEEDUP, //angular speed in radians per second
      flipFreq: 20*1000/SPEEDUP, //rotates 180 degrees every 20s
      randAngFreq: 5*1000/SPEEDUP, //rotates randomly every 5s
      randAngMag: 40*Math.PI/180, //how much it wiggles by for random rotations
      actAngle: 45*Math.PI/180 //how much it rotates upon activation
    };
    var OBST_SPEC = {
      N: 4, //number of obstacle roombas
      R: 1.5*ROOMBA_SPEC.R, //radius of the obstacle roombas
      S: ROOMBA_SPEC.S, //speed of the obstacle roombas
      initD: 0.25*DIMS[0],
      angSpd: 1.38*SPEEDUP //angular speed in radians per second
    };
    var POINTS_SPEC = {
      initScore: 12000,
      goal: 2000,
      miss: -1000,
      livingPenalty: -100, //per minute
    };
    var RL_RWD = {
      goal: 10,
      earlyExit: -10,
      successfulActivation: 1,
      costOfLiving: -0.1
    };
    var NUM_STATES = 2 + 4*(ROOMBA_SPEC.N + OBST_SPEC.N);

    /****************
     * working vars */
    var canvas, ctx;
    var uav, roombas, obstacles;
    var points, cumRwd, gameOver;
    var globalTime;
    var lastRenderTime;

    var env, spec, agent;

    /*************
     * constants */
    var CENTER = [DIMS[0]/2, DIMS[1]/2];

    /***********
     * objects */
    function UAV(pos, color) {
      this.position = pos.slice(0);
      this.color = color;
      this.direc = [0, 0];
      this.r = UAV_SPEC.R;
      this.s = UAV_SPEC.S;
    }
    UAV.prototype.move = function(action, dt) {
      if (action < 2 || action > roombas.length + 2) return;

      var roombaIdx = action - 2;
      var roomba = roombas[roombaIdx];
      if (!roomba.active) return;
      var dir = [
        roomba.position[0] - this.position[0],
        roomba.position[1] - this.position[1]
      ];
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
      this.r = ROOMBA_SPEC.R; //radius
      this.s = ROOMBA_SPEC.S; //speed

      this.t1 = 0; //time since last flip
      this.t2 = 0; //time since last wiggle
      this.angleLeftToMove = 0; //how much this UAV needs to rotate
      this.active = true; //whether or not it's still in play
    }
    Roomba.prototype.update = function(dt) {
      if (!this.active) return;

      this.move(dt);
      this.t1 += dt;
      if (this.t1 > ROOMBA_SPEC.randAngFreq) {
        this.t1 = 0;
        this.randomizeAngle();
      }
      this.t2 += dt;
      if (this.t2 > ROOMBA_SPEC.flipFreq) {
        this.t2 = 0;
        this.flip();
      }
    };
    Roomba.prototype.rotateByAngle = function(theta) {
      if (!this.active) return;

      var currAng = Math.atan2(this.direc[1], this.direc[0]);
      this.direc[0] = Math.cos(currAng + theta);
      this.direc[1] = Math.sin(currAng + theta);
    };
    Roomba.prototype.queueRotation = function(theta) {
      if (!this.active) return;

      this.angleLeftToMove = (this.angleLeftToMove+theta)%(2*Math.PI);
    };
    Roomba.prototype.move = function(dt, forceTranslation) {
      if (!this.active) return;

      if (this.angleLeftToMove === 0 || forceTranslation) {
        var ds = this.s * dt/1000;
        this.position[0] += ds * this.direc[0];
        this.position[1] += ds * this.direc[1];
      } else {
        var dtheta = ROOMBA_SPEC.angSpd * dt/1000;
        this.rotateByAngle(dtheta);
        this.angleLeftToMove -= dtheta;
        if (this.angleLeftToMove < 0) this.angleLeftToMove = 0;
      }
    };
    Roomba.prototype.flip = function() {
      if (!this.active) return;

      this.queueRotation(Math.PI);
    };
    Roomba.prototype.randomizeAngle = function() {
      if (!this.active) return;

      var thetaChange = -ROOMBA_SPEC.randAngMag/2;
      thetaChange += ROOMBA_SPEC.randAngMag*Math.random();
      this.rotateByAngle(thetaChange);
    };
    Roomba.prototype.distTo = function(ent) {
      return mag([
        this.position[0] - ent.position[0],
        this.position[1] - ent.position[1]
      ]);
    };
    Roomba.prototype.collideWith = function(roomba) {
      if (!this.active) return;

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
      if (!this.active) return;

      this.queueRotation(ROOMBA_SPEC.actAngle);
    };
    Roomba.prototype.deactivate = function() { //remove it from play
      this.active = false;
      this.position = [0, 0];
      this.direc = [0, 0];
      this.angleLeftToMove = 0;
    };

    function ObstacleRoomba(pos, color) {
      this.position = pos.slice(0);
      this.color = color;
      this.direc = norm([
        -this.position[1] + CENTER[1],
        this.position[0] - CENTER[0]
      ]); //perpendicular to initial position vector, clockwise
      this.r = OBST_SPEC.R; //radius
      this.s = OBST_SPEC.S; //speed

      this.angleLeftToMove = 0; //angle to pretend to rotate for synchronicity
    }
    ObstacleRoomba.prototype.move = function(dt) {
      if (this.angleLeftToMove !== 0) {
        var shadowTheta = OBST_SPEC.angSpd * dt/1000;
        this.angleLeftToMove -= shadowTheta;
        if (this.angleLeftToMove < 0) this.angleLeftToMove = 0;
      } else {
        var theta = Math.atan2(
          this.position[1] - CENTER[1],
          this.position[0] - CENTER[0]
        );
        var dtheta = (OBST_SPEC.S/OBST_SPEC.initD) * dt/1000;
        this.direc = [
          -Math.sin(theta+dtheta),
          Math.cos(theta+dtheta)
        ];
        this.position = [
          OBST_SPEC.initD*this.direc[1] + CENTER[1],
          -OBST_SPEC.initD*this.direc[0] + CENTER[0]
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

      //init starting conditions
      initGameState();

      //controls
      $s('#play-again-btn').addEventListener('click', function() {
        gameOver = false;
        initGameState();
        render();
      });

      //init the reinforcement learner
      env = {};
      env.getNumStates = function() { return NUM_STATES; };
      env.getMaxNumActions = function() { return 2 + ROOMBA_SPEC.N; };
      spec = {alpha: 0.1}; 
      agent = new RL.DQNAgent(env, spec);

      render();
    }

    function initGameState() {
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
      for (var ai = 0; ai < ROOMBA_SPEC.N; ai++) {
        var theta = ai*(2*Math.PI/ROOMBA_SPEC.N);
        var x = ROOMBA_SPEC.initD*Math.cos(theta);
        var y = ROOMBA_SPEC.initD*Math.sin(theta);
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
      for (var ai = 0; ai < OBST_SPEC.N; ai++) {
        var theta = ai*(2*Math.PI/OBST_SPEC.N);
        var x = OBST_SPEC.initD*Math.cos(theta);
        var y = OBST_SPEC.initD*Math.sin(theta);
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

      //misc
      points = POINTS_SPEC.initScore; //arbitrary initial score
      cumRwd = 0;
      $s('#rwd').innerHTML = '0';
      $s('#cum-rwd').innerHTML = '0';
      gameOver = false;
      $s('#play-again-btn-cont').style.display = 'none'; //hide it
    }

    function handleRoombaRoombaCollisions() {
      for (var ai = 0; ai < roombas.length; ai++) {
        if (!roombas[ai].active) continue;

        for (var bi = ai+1; bi < roombas.length; bi++) {
          if (!roombas[bi].active) continue;

          //collisions
          if (roombas[ai].distTo(roombas[bi]) < roombas[ai].r+roombas[bi].r) {
            roombas[ai].collideWith(roombas[bi]);
          }
        }
      }
    }

    function handleRoombaObstacleCollisions() {
      for (var ai = 0; ai < obstacles.length; ai++) {
        var obstacle = obstacles[ai];
        for (var bi = 0; bi < roombas.length; bi++) {
          if (!roombas[bi].active) continue;

          //collisions
          if (roombas[bi].distTo(obstacle) < roombas[bi].r + obstacle.r) {
            obstacle.collideWith(roombas[bi]);
          }
        }
      }
    }

    function handleUAVObstacleCollisions() {
      for (var ai = 0; ai < obstacles.length; ai++) {
        var obstacle = obstacles[ai];
        if (mag([
          uav.position[0] - obstacle.position[0],
          uav.position[1] - obstacle.position[1]
        ]) < uav.r + obstacle.r) {
          //the uav collided with the obstacle!
          points = 0;
          gameOver = true;
          handleEndBehavior();
        }
      }
    }

    function handleExitBehavior() {
      var netRwdFromExits = 0;
      for (var ai = 0; ai < roombas.length; ai++) {
        if (!roombas[ai].active) continue;
        
        //if a roomba leaves any of the edges, remove the roomba
        if (roombas[ai].position[0] < 0 || roombas[ai].position[0] > DIMS[0] ||
            roombas[ai].position[1] < 0 || roombas[ai].position[1] > DIMS[1]) {
          //special goal edge
          if (roombas[ai].position[1] < 0) {
            //make sure you negate the previous miss deduction!
            points += POINTS_SPEC.goal;
            netRwdFromExits += RL_RWD.goal;
          } else {
            points += POINTS_SPEC.miss;
            netRwdFromExits -= RL_RWD.earlyExit;
          }

          //remove it
          roombas[ai].deactivate();
          $s('#num-robots').innerHTML = roombas.length;
          ai--; //it'll get incremented and ai will remain the same
        }
      }
      return netRwdFromExits;
    }

    function attemptMagnetActivation() {
      var actRwd = 0;

      //get the closest roomba
      var closestRoomba = roombas.reduce(
        function(closest, curr, idx) {
          if (!curr.active) return closest;
          
          var dist = mag([
            curr.position[0] - uav.position[0],
            curr.position[1] - uav.position[1]
          ]);
          if (dist < closest[1]) return [idx, dist];
          else return closest;
        },
        [-1, Infinity] //idx of closest, distance
      );

      if (closestRoomba[0] === -1) return 0;

      //make sure the closest is close enough
      if (closestRoomba[1] < Math.abs(uav.r - roombas[closestRoomba[0]].r)) {
        //if it is, rotate it
        roombas[closestRoomba[0]].activateMagnet();
        actRwd += RL_RWD.successfulActivation;
      }

      return actRwd;
    }

    function handleEndBehavior() {
      console.log('Game ended');

      Crush.clear(ctx, 'rgba(0, 0, 0, 0.3)');
      $s('#play-again-btn-cont').style.display = 'block';
      
      //just restart the game immediately
      initGameState();
      render();
    }

    function getReward() {
      var maxPossibleDistSum = ROOMBA_SPEC.N*Math.sqrt(
        DIMS[0] * DIMS[0] +
        DIMS[1] * DIMS[1]
      );
      return -roombas.reduce(function(total, roomba) {
        if (!roomba.active) return total;

        return total + roomba.distTo(uav); 
      }, 0)/maxPossibleDistSum;
    }

    function handleUAVExit() {
      //exiting is bad
      if (uav.position[0] < 0 || uav.position[0] > DIMS[0] ||
          uav.position[1] < 0 || uav.position[1] > DIMS[1]) {
        return RL_RWD.earlyExit;
      } else {
        return 0;
      }
    }

    function render() {
      if (gameOver) return;

      //deal with time
      var t = +new Date();
      var dt = t - lastRenderTime;
      lastRenderTime = t;
      globalTime += dt*SPEEDUP;
      $s('#t').innerHTML = fmtTime(globalTime);

      //init rwd to 0
      var rwd = 0;

      //draw the board
      drawBoard();

      //draw the roomba positions
      roombas.filter(function(roomba) {
        return roomba.active;  
      }).map(drawEntity);

      //draw the obstacles
      obstacles.map(drawEntity);

      //render the UAV
      drawEntity(uav);

      //roomba-roomba collisions
      handleRoombaRoombaCollisions();

      //roomba-obstacle collisions
      handleRoombaObstacleCollisions();

      //uav collisions with the obstacles
      handleUAVObstacleCollisions();

      //roomba velocity updates
      roombas.filter(function(roomba) {
        return roomba.active;  
      }).map(function(roomba) {
        roomba.update(dt);
      });

      //handle no more roombas case
      if (roombas.filter(function(r){return r.active}).length === 0) {
        gameOver = true;
        return handleEndBehavior();
      }

      //obstacle velocity updates
      obstacles.map(function(obstacle) {
        obstacle.move(dt);
      });

      //remove the roombas that've crossed the wall
      rwd += handleExitBehavior();

      //get the action
      var state = getState();
      var action = agent.act(state);
      
      //perform the action
      if (action > 1) {
        //move the UAV
        uav.move(action, dt);
        var exitRwd = handleUAVExit();
        rwd += exitRwd;
        if (exitRwd < 0) {
          gameOver = true;
          agent.learn(RL_RWD.earlyExit);
          return handleEndBehavior();
        }
      } else if (action === 1) {
        //uav-roomba interaction
        rwd += attemptMagnetActivation();
      }

      //score updates
      points += (dt*SPEEDUP)/(60*1000)*POINTS_SPEC.livingPenalty;
      $s('#score').innerHTML = Math.round(points);

      //deal with reward
      rwd += getReward(); //reward due to heuristics and cost of living
      cumRwd += rwd;
      $s('#rwd').innerHTML = Math.round(100*rwd)/100;
      $s('#cum-rwd').innerHTML = Math.round(100*cumRwd)/100;

      //game went on too long
      if (globalTime > MAX_TIME) {
        gameOver = true;
        agent.learn(RL_RWD.earlyExit);
        return handleEndBehavior();
      } else {
        agent.learn(rwd);
      }

      requestAnimationFrame(render);
    }
    
    function getState() {
      var state = [];
      state.push(uav.position[0]/DIMS[0]);
      state.push(uav.position[1]/DIMS[1]);

      obstacles.map(function(obstacle) {
        state.push(obstacle.position[0]/DIMS[0]);
        state.push(obstacle.position[1]/DIMS[1]);
        if (obstacle.angleLeftToMove === 0) {
          state.push(obstacle.direc[0]);
          state.push(obstacle.direc[1]);
        } else {
          state.push(0);
          state.push(0);
        }
      });

      roombas.map(function(roomba) {
        state.push(roomba.position[0]/DIMS[0]);
        state.push(roomba.position[1]/DIMS[1]);
        if (roomba.angleLeftToMove === 0) {
          state.push(roomba.direc[0]);
          state.push(roomba.direc[1]);
        } else {
          state.push(0);
          state.push(0);
        }
      });

      return state;
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

    function drawBoard() {
      Crush.clear(ctx, 'white');

      //draw the grid
      var thk = 2; //thickness in px
      ctx.fillStyle = '#F3F3F3';
      var xInc = (DIMS[0] - thk)/(NUM_GRID_LINES[0]-1);
      for (var ai = 0; ai < NUM_GRID_LINES[0]; ai++) {
        ctx.fillRect(ai*xInc, 0, thk, DIMS[1]);
    	}
      var yInc = (DIMS[1] - thk)/(NUM_GRID_LINES[1]-1);
      for (var ai = 0; ai < NUM_GRID_LINES[1]; ai++) {
        ctx.fillRect(0, ai*yInc, DIMS[0], thk);
    	}

      //draw the goal
      ctx.fillStyle = '#72DE2A';
      ctx.fillRect(0, 0, DIMS[0], 2*thk);
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

    function $s(id) { //for convenience
      if (id.charAt(0) !== '#') return false;
      return document.getElementById(id.substring(1));
    }

    return {
      init: initIARCSim
    };
})();

window.addEventListener('load', IARCSim.init);
