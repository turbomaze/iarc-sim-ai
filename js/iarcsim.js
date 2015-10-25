/******************\
| IARC Simulation  |
| @author Anthony  |
| @version 0.1     |
| @date 2015/10/24 |
| @edit 2015/10/24 |
\******************/

var IARCSim = (function() {
    'use strict';

    /**********
     * config */
    var DIMS = [500, 500];
    var N = 10;

    /****************
     * working vars */
    var canvas, ctx;
    var roombas;
    var lastRenderTime;

    /*************
     * constants */
    var CENTER = [DIMS[0]/2, DIMS[1]/2];
    var R = 7; //radius of the Roombas
    var S = 20; //px per second
    var INIT_D = 50;

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
    }

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
      roombas.map(function(roomba) {
        updateRoomba(roomba, dt);
      });

      requestAnimationFrame(render);
    }

    /********************
     * helper functions */
    function norm(a) {
      var mag = Math.sqrt(a.reduce(function(acc, curr) {
        return acc + curr*curr;
      }, 0));
      return a.map(function(comp) {
        return comp/mag;
      });
    }

    function updateRoomba(roomba, time) {
      var ds = roomba.s * time/1000;
      roomba.position[0] += ds * roomba.direc[0];
      roomba.position[1] += ds * roomba.direc[1];
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
