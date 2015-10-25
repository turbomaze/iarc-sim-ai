/******************\
| IARC Simulation  |
| @author Anthony  |
| @version 0.2.0.1 |
| @date 2015/10/24 |
| @edit 2015/10/24 |
\******************/

var IARCSim = (function() {
    'use strict';

    /**********
     * config */
    var DIMS = [500, 0];
    DIMS[1] = DIMS[0]; //force squareness
    var N = 10; //number of roombas
    var SPEEDUP = 10;
    var FLIP_FREQ = 20*1000/SPEEDUP; //every 20s
    var RAND_ANG_FREQ = 5*1000/SPEEDUP; //every 5s
    var DTHETA = 40*Math.PI/180; //how much it wiggles by

    /****************
     * working vars */
    var canvas, ctx;
    var roombas;
    var lastRenderTime;

    /*************
     * constants */
    var CENTER = [DIMS[0]/2, DIMS[1]/2];
    var R = 0.0085*DIMS[0]; //radius of the Roombas
    var S = 0.0165*DIMS[0]*SPEEDUP; //px per second
    var INIT_D = 0.05*500;

    /***********
     * objects */
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
    Roomba.prototype.move = function(dt) {
      var ds = this.s * dt/1000;
      this.position[0] += ds * this.direc[0];
      this.position[1] += ds * this.direc[1];
    };
    Roomba.prototype.flip = function() {
      this.rotateByAngle(Math.PI);
    };
    Roomba.prototype.randomizeAngle = function() {
      var thetaChange = -DTHETA/2 + DTHETA*Math.random();
      this.rotateByAngle(thetaChange);
    };

    /******************
     * work functions */
    function initIARCSim() {
      canvas = $s('#canvas');
      canvas.width = DIMS[0];
      canvas.height = DIMS[1];
      ctx = canvas.getContext('2d');

      roombas = [];
      for (var ai = 0; ai < N; ai++) {
        var theta = ai*(2*Math.PI/N);
        var x = INIT_D*Math.cos(theta);
        var y = INIT_D*Math.sin(theta);
        roombas.push(
          new Roomba([CENTER[0]+x, CENTER[1]+y], 'red')
        );
      }

      lastRenderTime = +new Date();
      render();
    }

    function render() {
      clearCanvas();

      var t = +new Date();
      var dt = t - lastRenderTime;
      lastRenderTime = t;

      roombas.map(drawRoomba);
      for (var ai = 0; ai < roombas.length; ai++) {
        for (var bi = ai+1; bi < roombas.length; bi++) {
          if (mag([
            roombas[ai].position[0] - roombas[bi].position[0],
            roombas[ai].position[1] - roombas[bi].position[1]
          ]) <= roombas[ai].r + roombas[bi].r) { //colliding
            roombas[ai].flip();
            roombas[ai].move(1000/60); //assume 60fps
            roombas[bi].flip();
            roombas[bi].move(1000/60); //assume 60fps
          }
        }
      }
      roombas.map(function(roomba) {
        roomba.update(dt);
      });

      requestAnimationFrame(render);
    }

    /********************
     * helper functions */
    function mag(a) {
      return Math.sqrt(a.reduce(function(acc, curr) {
        return acc + curr*curr;
      }, 0));
    }
    function norm(a) {
      var magnitude = mag(a);
      return a.map(function(comp) {
        return comp/magnitude;
      });
    }

    function drawRoomba(roomba) {
      //represent the Roomba
      Crush.drawPoint(ctx, roomba.position, R, roomba.color);
      //velocity
      Crush.drawLine(ctx, roomba.position, [
        roomba.position[0]+2*roomba.r*roomba.direc[0],
        roomba.position[1]+2*roomba.r*roomba.direc[1]
      ], 'red', 2);
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
      init: initIARCSim
    };
})();

window.addEventListener('load', IARCSim.init);
