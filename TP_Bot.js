// ==UserScript==
// @name        Labyrinth Bot
// @version     0.1
// @include     *.newcompte.fr:*
// ==/UserScript==

var obstacleData;   // data to be pulled in from Obstacles.json

function waitForId(fn) {
    // Don't execute the function until tagpro.playerId has been assigned.
    if (!tagpro || !tagpro.playerId) {
        return setTimeout(function() {
            waitForId(fn);
        }, 100);
    } else {
        // Only run the script if we are not spectating.
        if (!tagpro.spectator) {
            fn();
        }
    }
}

function script() {
    let ctrl_period = 50;
    let counter = 0;
    let debugr = create_debugger(); // draws point of where bot is headed to
    let buttonDist = 25;
    let moveDist = 20;
    let standDist = 3;

    let obstacles = preprocess();
    // console.log(obstacles)

    function create_decision_maker(me){
        var decision_maker = {};
        decision_maker.me = me;
        decision_maker.role = {state: 'follow', step: 0, role: false};
        decision_maker.human = null;         // will be set for caching
        decision_maker.lastPlayerPos = {x: me.x, y: me.y, vx: -0, vy: -0};
        decision_maker.obsKey = '0';         // key of current obstacle
        decision_maker.obsData = {}          // data of current obstacle
        decision_maker.isAlongPath = false;  // keep track of if the bot is currently moving along a path
        decision_maker.currTargetIndex = 0;  // current target on path
        decision_maker.path = [];            // [[x1, y1], [x2, y2], ...] of path
        decision_maker.startedPath = false;

        decision_maker.decide = function(){
            this.decide_state();
            return this.create_target();
        }

        // Based off current state and step of bot, decide where it should go
        decision_maker.create_target = function(){
            var curPlayerPos = {};
            var target = this.lastPlayerPos;
            if (!this.isAlongPath) {
                if (this.role.state === 'follow') {    // bot should follow around human player
                    if (!this.human) {
                        let humanPlayers = Object.keys(tagpro.players).filter(x => {
                            // return tagpro.players[x].name === 'Player 1';   // Hardcoding that human player has to be named 'Player 1'
                            return x !== tagpro.playerId;
                        })
                        if (humanPlayers.length > 0) {
                            this.human = tagpro.players[humanPlayers[0]];
                            this.lastPlayerPos = {
                              x: this.human.x, y: this.human.y, vx: -0, vy: -0
                            };    // initialize last and current player position once human player has been found
                        }
                    }
                    if (this.human) {
                      //get player's current position and compare it to the last recorded position
                      //if the difference is >= the size of a ball, move the last recorded position and update it as the player's current position
                        curPlayerPos = {
                            x: this.human.x, y: this.human.y, vx: -0, vy: -0
                        };

                        var dist = Math.sqrt(Math.pow(curPlayerPos.x-this.lastPlayerPos.x, 2) + Math.pow(curPlayerPos.y-this.lastPlayerPos.y, 2));
                        if (Math.abs(dist) >= 75){
                          target = this.lastPlayerPos;
                          this.lastPlayerPos = curPlayerPos;
                        }
                    }

                } else if (this.role.state === 'Basic Button') {
                    var x=this.me.x, y=this.me.y;
                    switch (this.role.step) {
                        case 0:      // bot should go to button1
                            x = this.obsData.positions.button1[0] * 40;
                            y = this.obsData.positions.button1[1] * 40;
                            break;
                        case 1:      // bot should go to button2
                            x = this.obsData.positions.button2[0] * 40;
                            y = this.obsData.positions.button2[1] * 40;
                            break;
                        case 2:     // bot should wait
                            break;
                        case 3:     // bot should go to goal
                            x = this.obsData.positions.goal[0] * 40;
                            y = this.obsData.positions.goal[1] * 40;
                            break;
                        default:
                            break;
                    }
                    target = {x: x, y: y, vx: -0, vy: -0};
                } else if (this.role.state === 'Trust') {
                    var x=this.mex, y=this.me.y;
                    switch (this.role.step) {
                        case 0:      // bot should go to waiting area
                            let path = [];
                            path.push(this.obsData.positions.topPos);
                            path.push(this.obsData.positions.interPos1);
                            path.push(this.obsData.positions.waitPos1);
                            this.startedPath = true;
                            this.startPath(path);
                            break;
                        case 1:      // waiting on boost
                            target = {x: this.me.x, y: this.me.y, vx: -this.human.vx, xy: 0};
                            break;
                        case 2:      // go to goal
                            target = {x: this.obsData.positions.goal[0]*40, y: this.obsData.positions.goal[1]*40, vx: -0, vy: -0};
                            break;
                        default:
                            break;
                    }
                }
            } else {   // move along a path
                let isDone = this.moveAlongPath();
                if (!isDone) {
                    var [x, y] = this.path[this.currTargetIndex];
                    target = {x: x*40, y: y*40, vx: -0, vy: -0};
                }
            }

            // highlights
            debugr.draw_point(target.x, target.y);

            return target;
        }

        // Check if we've entered an obstacle
        decision_maker.decide_state = function(){
            if (this.role.state === 'follow') {
                for (let o in obstacles) {
                    let obstacle = obstacles[o];
                    let {x1, y1, x2, y2} = obstacle.dim;
                    if (this.me.x >= x1 && this.me.x <= x2 && this.me.y >= y1 && this.me.y <= y2) {  // We are inside an obstacle
                        this.obsKey = o;
                        this.obsData = obstacle;
                        this.set_state(obstacle.name, 0, false);   // set state to be name of the obstacle
                        return;
                    }
                }
            } else {
                this.decide_step();
            }
        }

        // Were inside an obstacle. Decide if we've exited, otherwise decide step
        decision_maker.decide_step = function(){
            let obstacle = obstacles[this.obsKey];
            if (this.role.state === 'Basic Button') {
                let {x1, y1, x2, y2} = obstacle.dim;
                if (this.me.x < x1 || this.me.x > x2 || this.me.y < y1 || this.me.y > y2) {  // we're outside obstacle
                    this.set_state('follow', 0, false);
                } else {
                    let isHumanPastGate = this.human.y + 50 < this.obsData.positions.gatePos[1] * 40 ? true : false;
                    let isBotPastGate = this.me.y + 50 < this.obsData.positions.gatePos[1] * 40 ? true : false;
                    if (isHumanPastGate && isBotPastGate) {  // check if both players above gate
                        this.set_state(this.role.state, 3, false);
                    } else {
                        let [i, y] = this.obsData.positions.gatePos;
                        let isGateOpen = tagpro.map[i][y] === 9.1 ? false : true;
                        if (!isGateOpen) {   // gate is not open
                            if (!isHumanPastGate && !isBotPastGate) {  // human and bot both behind gate
                                this.set_state(this.role.state, 0, false);
                            } else if (isBotPastGate && !isHumanPastGate) { // bot ahead, human behind
                                this.set_state(this.role.state, 1, false);
                            } else {
                                this.set_state(this.role.state, 2, false); // bot should wait for human to open gate
                            }
                        } else {
                            if (this.isOnTile(false, this.obsData.positions.button2, 25)) {  // human is on button 2
                                this.set_state(this.role.state, 1, false);
                            } else if (this.isOnTile(true, this.obsData.positions.button1, 25)) { // bot is on button1, dont move
                                this.set_state(this.role.state, 2, false);
                            } else if (this.isOnTile(true, this.obsData.positions.button2, 25)) { // bot is on button2
                                this.set_state(this.role.state, 2, false);
                            } else {
                                this.set_state(this.role.state, 1, false);
                            }
                        }
                    }
                }
            } else if (this.role.state === 'Trust') {
                if (this.human.flag || this.me.flag && !this.role.step === 2) {   // if either player has flag, go to goal
                    this.set_state(this.role.state, 2, false); 
                } else if (this.role.step !== 2) {
                    if (this.role.step === 0) {    // start of obstacle, go to waiting spot 
                        if (this.startedPath && !this.isAlongPath) {      // finished path, moment of truth 
                            this.startedPath = false;
                            this.set_state(obstacle.name, 1, false);
                        }
                    }
                }
            }
        }

        // Helper function to set step, state the bot is in
        decision_maker.set_state = function(state, step, role){
            this.role.state = state;
            this.role.step = step;
            this.role.role = role;
        }

        // decide if a player is sitting on a tile
        // params:
        //    bot: false = check if bot is on tile, true = check if human on tile
        //    tile: [x, y] of tile
        //    dist = accuracy of check
        decision_maker.isOnTile = function(isBot, tile, dist=3) {
            let {x, y} = isBot ? this.me : this.human;  // x and y coords of player
            x; y;
            let tx = tile[0] * 40;
            let ty = tile[1] * 40;
            let d = Math.sqrt(Math.pow(tx-x, 2) + Math.pow(ty-y, 2));
            return d <= dist;
        }

        // start a path
        decision_maker.startPath = function(path) {
            this.path = path;
            this.currTargetIndex = 0;
            this.isAlongPath = true;
        }

        // move bot along a path
        decision_maker.moveAlongPath = function() {
            var dist = 30;
            if (this.currTargetIndex == this.path.length - 1) {
                    dist = 3;
            }
            let currTarget = this.path[this.currTargetIndex];
            if (this.isOnTile(true, currTarget, dist)) {
                this.currTargetIndex++;
                if (this.currTargetIndex >= this.path.length) {  // finished the path!
                    this.path = [];
                    this.isAlongPath = false;
                    this.currTargetIndex = 0;
                    return true;
                }
            }
            return false;
        }

        return decision_maker;
    }

    function run_bot() {
        var me = tagpro.players[tagpro.playerId];
        // no_draw();

        PID_constants = {};
        PID_constants.KP = 0.036;
        PID_constants.KI = 0.011;
        PID_constants.KD = 3.5;
        PID_constants.KS = 11;

        var controller = create_controller(PID_constants);
        var decision_maker = create_decision_maker(me);

        setInterval(function() {
            // determine a target state
            var target = decision_maker.decide();
            // execute the control to get to the target
            controller.execute(me,target);
            // increment step counter
            counter++;
        }, ctrl_period);
    }

    function main(fn) {
        fn();
    }

    main(run_bot);
}


// Find and preprocess all obstacles in maze
function preprocess() {
    let obstacles = {};
    let obstacleStartingPoints = [];
    for (let i=0; i<tagpro.map.length; i++) {
        for (let y=0; y<tagpro.map[i].length; y++) {
            if (tagpro.map[i][y] === 18) {
                obstacleStartingPoints.push([i, y]);
            }
        }
    } 
    for (let point of obstacleStartingPoints) {
        let [i, y] = point;
        let config = [
            tagpro.map[i-1].slice(y-1, y+2).map(x => {return x === 9 ? 1 : 0}),
            tagpro.map[i].slice(y-1, y+2).map(x => {return x === 9 ? 1 : 0}),
            tagpro.map[i+1].slice(y-1, y+2).map(x => {return x === 9 ? 1 : 0})
        ]
        for (let key in obstacleData) {
            let obstcl = obstacleData[key];
            var isSame = true;
            for (let a=0; a<3; a++) {
                for (let b=0; b<3; b++) {
                    if (config[a][b] !== obstcl.config[a][b]) isSame = false;
                }
            }
            if (isSame) {  // found the obstacle type
                let obstclData = {};
                obstclData.startLoc = point;  // start of maze
                obstclData.positions = {};
                obstclData.dim = {
                    x1: (i + obstcl.topLeftOffset[0]) * 40,   // top left of obstacle
                    y1: (y + obstcl.topLeftOffset[1]) * 40,
                    x2: (i + obstcl.botRightOffset[0]) * 40,  // bot right of obstacle
                    y2: (y + obstcl.botRightOffset[1]) * 40
                }
                obstclData.name = obstcl.name;
                if (obstclData.name === 'Basic Button') {
                    obstclData.positions.gatePos = [point[0], point[1] - 8];
                    obstclData.positions.button1 = [point[0] - 2, point[1] - 5];
                    obstclData.positions.button2 = [point[0] + 2, point[1] - 11];
                    obstclData.positions.goal = [point[0] + obstcl.goalOffset[0], point[1] + obstcl.goalOffset[1]];
                } else if (obstclData.name === 'Trust') {
                    obstclData.positions.waitPos2 = [point[0] + 8, point[1] + 7]; // left square next to boost
                    obstclData.positions.waitPos1 = [point[0] - 8, point[1] + 7]; // right square next to boost
                    obstclData.positions.topPos = [point[0], point[1] + 3];         // top of obstacle
                    obstclData.positions.interPos2 = [point[0] + 7, point[1] + 4];  // top right of obstacle
                    obstclData.positions.interPos1 = [point[0] - 7, point[1] + 4];  // top left of obstacle
                    obstclData.positions.goal = [point[0] + obstcl.goalOffset[0], point[1] + obstcl.goalOffset[1]];
                } else if (obstacleData.name === 'Mars') {

                }
                obstacles[obstcl.name] = obstclData;
                break;
            }
        }
    }
    return obstacles;
}

tagpro.ready(function() {
    let url = 'https://raw.githubusercontent.com/MarksCode/CM146-Final-Project/master/Obstacles.json';
    fetch(url)
    .then(response =>
        response.json().then(data => ({
            data: data,
            status: response.status
        })
    ).then(res => {
        obstacleData = res.data;
        waitForId(script);
    }));
});




/*********************************/
/*                               */
/* Stuff we probably won't touch */
/*                               */
/*********************************/


/**
 *  All functionality related to steering
 */
function create_controller(PIDconstants){
    var controller = {};
    controller.output = {};
    controller.output.threshold = 0.18;
    controller.output.set = function(u,me){
        this.horz(u.x,me);
        this.vert(u.y,me);
    };

    controller.output.horz = function(x,me){
        // left/right control
        if (x > this.threshold) { // go right
            tagpro.sendKeyPress('left', true);
            tagpro.sendKeyPress('right', false);
            me.right = true;
            me.left = false;
        } else if (x < -this.threshold) { // go left
            tagpro.sendKeyPress('right', true);
            tagpro.sendKeyPress('left', false);
            me.right = false;
            me.left = true;
        } else { // release both
            tagpro.sendKeyPress('right', true);
            tagpro.sendKeyPress('left', true);
            me.right = false;
            me.left = false;
        }
    }

    controller.output.vert = function(y,me){
        // up/down control
        if (y < -this.threshold) { // go down
            tagpro.sendKeyPress('up', true);
            tagpro.sendKeyPress('down', false);
            me.up = false;
            me.down = true;
        } else if (y > this.threshold) { // go up
            tagpro.sendKeyPress('up', false);
            tagpro.sendKeyPress('down', true);
            me.up = true;
            me.down = false;
        } else { // release both
            tagpro.sendKeyPress('up', true);
            tagpro.sendKeyPress('down', true);
            me.up = false;
            me.down = false;
        }
    }

    controller.execute = function(me,target){
        ctrl_vec = this.PID.get_ctrl_vec(me,target);
        this.output.set(ctrl_vec,me);
    }

    function create_PID(constants){
        var PID = {};
        PID.dims = {};
        PID.dims.list = ['x','y'];
        PID.dims.num = PID.dims.list.length;
        PID.constants = {};
        PID.constants.set_constants = function(constants){
            for (var key in constants){
                if (constants.hasOwnProperty(key)) {
                    this[key] = constants[key];
                }
            }
        }
        PID.constants.set_constants(constants);
        PID.accum_error = {};
        PID.reset_accum_error = function(){
            PID.accum_error.pos = {'x':0,'y':0};
            PID.accum_error.vel = {'x':0,'y':0};
        };
        PID.reset_accum_error();
        PID.errors = {}; // gets set before its read
        PID.get_ctrl_vec = function(state_current, state_goal){
            this.errors = this.calc_error(state_current,state_goal);
            this.accumulate_error(this.errors);
            var u = {};
            for (var i = 0; i < this.dims.num; i++){
                var dim = this.dims.list[i];
                u[dim] = this.calc_P(dim) + this.calc_I(dim) + this.calc_D(dim);
            }
            return u;
        };
        PID.calc_P = function(axis){
            return this.constants.KP*this.errors.pos[axis];
        }
        PID.calc_I = function(axis){
            return this.constants.KI*this.accum_error.pos[axis];
        };
        PID.calc_D = function(axis){
            return this.constants.KD*this.errors.vel[axis];
        };
        PID.calc_error = function(state_current, state_goal){
            var errors = {};
            errors.pos = {};
            errors.vel = {};
            errors.pos.x = state_goal.x - state_current.x;
            errors.pos.y = state_current.y - state_goal.y;
            errors.vel.x = state_goal.vx - state_current.vx;
            errors.vel.y = state_current.vy - state_goal.vy;
            return errors;
        };
        PID.accumulate_error = function(errors){
            this.accum_error.pos.x += errors.pos.x;
            this.accum_error.pos.y += errors.pos.y;
            if (this.accum_error.pos.x > this.constants.KS) this.accum_error.pos.x = this.constants.KS;
            if (-this.accum_error.pos.x > this.constants.KS) this.accum_error.pos.x = -this.constants.KS;
            if (this.accum_error.pos.y > this.constants.KS) this.accum_error.pos.y = this.constants.KS;
            if (-this.accum_error.pos.y > this.constants.KS) this.accum_error.pos.y = -this.constants.KS;
        };

        return PID;
    }

    controller.PID = create_PID(PIDconstants);
    return controller;
}

// draw a rectange on map at specified location
function create_debugger() {
    var debug = {};
    var layer = tagpro.renderer.layers.foreground;
    var point = new PIXI.Graphics();
    layer.addChild(point);
    debug.draw_point = function(x, y, size=5) {
        point.clear();
        point.lineStyle(2, 0xFF0000);
        point.drawCircle(x, y, size);
    }
    return debug;
}

// removes the all tagpro rendering and suppresses errors to
// preserve computational power for actual bot shenanigans
function no_draw() {
    window.onerror = function(message, url, lineNumber) {
        return true; // prevents browser error messages
    };
    // reserve bot resources: don't render anything
    delete tagpro.renderer;
    tagpro.renderer = {};
    tagpro.renderer.destroyPlayer = function(e) {
        delete tagpro.players[e.id];
        return true;
    };
    tagpro.renderer.drawName = function(e,t) {
        return true;
    };
    tagpro.sound = false;
    delete tagpro.sounds;
    delete tagpro.soundTiles;
    $('html').remove();
}

// Overriding this function to get a more accurate velocity of players.
// Velocity is saved in player.vx and vy.
Box2D.Dynamics.b2Body.prototype.GetLinearVelocity = function() {
    tagpro.players[this.player.id].vx = this.m_linearVelocity.x;
    tagpro.players[this.player.id].vy = this.m_linearVelocity.y;
    return this.m_linearVelocity;
};
