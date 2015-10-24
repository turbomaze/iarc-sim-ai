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

    /*************
     * constants */
    var CENTER = [DIMS[0]/2, DIMS[1]/2];
    var R = 7;
    var INIT_D = 50;

    /***********
     * objects */
    function Roomba(pos, color) {
      this.position = pos.slice(0);
      this.color = color;
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

      render();
    }

    function render() {
      clearCanvas();

      roombas.map(drawRoomba);
    }

    /********************
     * helper functions */
    function drawRoomba(roomba) {
      Crush.drawPoint(ctx, roomba.position, R, roomba.color);
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
