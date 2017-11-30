// ==UserScript==
// @name         Labyrinth Bot
// @version      0.1
// @include      *.newcompte.fr:*
// ==/UserScript==

function waitForId(fn) {
    // Don't execute the function until tagpro.playerId has been assigned.
    if (!tagpro || !tagpro.playerId) {
        return setTimeout(function() {
            waitForId(fn)
        }, 100);
    } else {
        // Only run the script if we are not spectating.
        if (!tagpro.spectator) {
            fn();
        }
    }
}

function script() {
    var ctrl_period = 50;
    var counter = 0;

    // Overriding this function to get a more accurate velocity of players.
    // Velocity is saved in player.vx and vy.
    Box2D.Dynamics.b2Body.prototype.GetLinearVelocity = function() {
        tagpro.players[this.player.id].vx = this.m_linearVelocity.x;
        tagpro.players[this.player.id].vy = this.m_linearVelocity.y;
        return this.m_linearVelocity;
    };

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

    function create_vec2(x,y){
        var vec2 = {};
        vec2.x = x;
        vec2.y = y;
        vec2.dot = function(other){
            return this.x * other.x + this.y * other.y;
        }
        vec2.cross = function(other){
            // returns scalar, which is really a vector along z
            return this.x * other.y - this.y * other.x;
        }
        vec2.norm = function(){
            return Math.sqrt(this.dot(this));
        }
        vec2.scale = function(factor){
            return create_vec2(this.x*factor,this.y*factor);
        }
        vec2.unit = function(){
            var norm = this.norm();
            return create_vec2(this.x,this.y).scale(1/norm);
        }
        vec2.perp = function(){
            return create_vec2(-this.y,this.x);
        }
        vec2.neg = function(){
            return create_vec2(-this.x,-this.y);
        }
        vec2.add = function(other){
            return create_vec2(this.x + other.x, this.y + other.y);
        }
        vec2.sub = function(other){
            return this.add(other.neg());
        }
        vec2.lr = function(p2,p3){
            return (p2.x * p3.y - p2.y * p3.x) - this.x * (p3.y - p2.y) + this.y * (p3.x - p2.x);
        }

        return vec2;
    }

    function create_rectangle(vec2_array){
        var rectangle = {};
        rectangle.points = vec2_array;
        rectangle.contains = function(m){
            var ab = this.points[0][0].sub(this.points[0][1]); // b - a
            var bc = this.points[1][0].sub(this.points[0][0]); // c - b
            var am = m.sub(this.points[0][1]);
            var bm = m.sub(this.points[0][0]);
            return (0 <= ab.dot(am) && ab.dot(am) <= ab.dot(ab) && 0 <= bc.dot(bm) && bc.dot(bm) <= bc.dot(bc));
        }

        return rectangle;
    }

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
                tagpro.sendKeyPress("left", true);
                tagpro.sendKeyPress("right", false);
                me.right = true;
                me.left = false;
            } else if (x < -this.threshold) { // go left
                tagpro.sendKeyPress("right", true);
                tagpro.sendKeyPress("left", false);
                me.right = false;
                me.left = true;
            } else { // release both
                tagpro.sendKeyPress("right", true);
                tagpro.sendKeyPress("left", true);
                me.right = false;
                me.left = false;
            }
        }

        controller.output.vert = function(y,me){
            // up/down control
            if (y < -this.threshold) { // go down
                tagpro.sendKeyPress("up", true);
                tagpro.sendKeyPress("down", false);
                me.up = false;
                me.down = true;
            } else if (y > this.threshold) { // go up
                tagpro.sendKeyPress("up", false);
                tagpro.sendKeyPress("down", true);
                me.up = true;
                me.down = false;
            } else { // release both
                tagpro.sendKeyPress("up", true);
                tagpro.sendKeyPress("down", true);
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

    function create_decision_maker(me){
        var decision_maker = {};
        decision_maker.me = me;
        decision_maker.role = {};
        // decision_maker.behavior = 'sit';

        decision_maker.decide = function(){
            this.decide_role();
            return this.create_target();
        }

        decision_maker.find_player = function(criterion_fn){
            for (var key in tagpro.players) {
                if (tagpro.players.hasOwnProperty(key)) {
                    // criterian_fn must take decision_maker and a player as arguments
                    if (criterion_fn(this,tagpro.players[key])) {
                        return tagpro.players[key];
                    }
                }
            }
        }

        decision_maker.create_target = function(){
            var target = {};
            var ofm_center = [12,9];
            var center = create_vec2(ofm_center[0],ofm_center[1]).scale(40);
            // var opponent = this.find_player(function(dm,player){
            //     return (player.id !== dm.me.id);
            // });

            // defense
            // if (this.role.pos === 'd'){
            //     target = opponent;
            // }

            return target;
        }

        decision_maker.decide_role = function(){
            this.set_role('d',0);
        }

        decision_maker.set_role = function(position,aggression){
            this.role.pos = position;
        }

        return decision_maker;
    }

    function run() {
        var me = tagpro.players[tagpro.playerId];
        no_draw();

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
            target = {x: 400, y: 400, vx: -0, vy: -0}  /* This is what target looks like */
            // execute the control to get to the target
            controller.execute(me,target);
            // increment step counter
            counter++;
        }, ctrl_period);
    }

    function main(fn) {
        fn();
    }

    main(run);
}

tagpro.ready(function() {
    waitForId(script);
});