use macroquad::prelude::*;

pub struct MouseMovement {
    pub last_mousex: f32,
    pub last_mousey: f32,

    pub mousex: f32,
    pub mousey: f32,

    pub dx: f32,
    pub dy: f32,
}

impl MouseMovement {
    pub fn new() -> Self {
        let (mx, my) = mouse_position();

        Self {
            last_mousex: mx,
            last_mousey: my,
            mousex: mx,
            mousey: my,
            dx: 0.0,
            dy: 0.0,
        }
    }

    pub fn update(&mut self) {
        let (mx, my) = mouse_position();
        //let mpos = camera.world_to_screen( vec2(mx, my) );
        //let (mx, my) = (mpos.x, mpos.y);

        //prohoď
        self.last_mousex = self.mousex;
        self.last_mousey = self.mousey;

        self.mousex = mx;
        self.mousey = my;

        self.dx = self.mousex - self.last_mousex;
        self.dy = self.mousey - self.last_mousey;

        self.dx *= 2.0; //TODO: fixni to dpi ve více rozumném způsobu (pokud za to vůbec může dpi...)
        self.dy *= 2.0; //TODO: fixni to dpi ve více rozumném způsobu (pokud za to vůbec může dpi...)
                        //nemůže, může za to ten fakt, že máme rozsah -1..1 a ne 0..1, což má celkem délku 2 a ne 1
    }
}
