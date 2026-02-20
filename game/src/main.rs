#![no_std]
#![no_main]

extern crate alloc;
use alloc::vec::Vec;
use alloc::rc::Rc;

use core::mem::MaybeUninit;
use uninit::uninit_array;

use psp::sys::*;
use psp_mesh_writer::prelude::*;

mod display;
use display::*;

mod mem;
use mem::*;

mod model_instance;
use model_instance::{Drawable, ModelInstance};

mod timer;
use timer::*;

psp::module!("game", 1, 1);

struct GraphicsBuffer {
    display_list: Align64<[MaybeUninit<u8>; 0x10000]>,
}

impl Default for GraphicsBuffer {
    fn default() -> Self {
        Self {
            display_list: Align64(uninit_array![u8; 0x10000]),
        }
    }
}

struct Gizmo {
    vertex_type: VertexType,
    vertex_count: i32,
    vertex_buffer: Align16<[u8; 0xFF]>,
    morph_weight: f32,
}

impl Gizmo {
    fn new() -> Result<Self, psp_mesh_writer::Error> {
        let mesh_description = MeshDescriptionBuilder::new(PositionFormat::I16)
            .color_format(ColorFormat::A8B8G8R8)
            .morph_count(COUNT_2)
            .build();
        let vertex_type = VertexType::from_bits_truncate(mesh_description.clone().flags() as i32);
        let mut vertex_buffer = Align16([0u8; 0xFF]);
        let vertex_count = MeshWriter::new(mesh_description, &mut vertex_buffer.0)
            .colors_morph(&[
                [0xff0000ff_u32, 0xffffff00_u32],
                [0xff0000ff_u32, 0xffffff00_u32],
                [0xff00ff00_u32, 0xffff00ff_u32],
                [0xff00ff00_u32, 0xffff00ff_u32],
                [0xffff0000_u32, 0xff00ffff_u32],
                [0xffff0000_u32, 0xff00ffff_u32],
            ])?
            .positions_morph(&[
                [[i16::MAX, 0, 0], [i16::MIN, 0, 0]],
                [[i16::MIN, 0, 0], [i16::MAX, 0, 0]],
                [[0, i16::MAX, 0], [0, i16::MIN, 0]],
                [[0, i16::MIN, 0], [0, i16::MAX, 0]],
                [[0, 0, i16::MAX], [0, 0, i16::MIN]],
                [[0, 0, i16::MIN], [0, 0, i16::MAX]],
            ])?
            .advance_vertex(6)?
            .tell_vertex() as i32;

        Ok(Self {
            vertex_type,
            vertex_count,
            vertex_buffer,
            morph_weight: 0.0,
        })
    }

    fn draw(&mut self, delta: f32) {
        unsafe {
            self.morph_weight += delta;
            self.morph_weight %= 2.0;

            let weight = if self.morph_weight > 1.0 {
                1.0 - (self.morph_weight - 1.0)
            } else {
                self.morph_weight
            };

            sceGuDisable(GuState::Texture2D);
            sceGuModelColor(0x00000000, 0xffffffff, 0xffffffff, 0xffffffff);

            sceGuMorphWeight(0, weight);
            sceGuMorphWeight(1, 1.0 - weight);

            sceGumLoadIdentity();
            sceGumTranslate(&ScePspFVector3 {
                x: 0.0,
                y: 0.0,
                z: -5.0,
            });
            sceGumRotateXYZ(&ScePspFVector3 {
                x: 45_f32.to_radians(),
                y: 45_f32.to_radians(),
                z: 0_f32.to_radians(),
            });

            sceGumDrawArray(
                GuPrimitive::Lines,
                self.vertex_type,
                self.vertex_count,
                core::ptr::null(),
                self.vertex_buffer.0.as_ptr() as _,
            );
        }
    }
}

struct World
{
    models: Vec<Rc<psp_file_formats::Model>>,
    instances: Vec<ModelInstance>,
    gizmo: Option<Gizmo>,
    timer: Timer,
}

impl World {
    fn new() -> Self {
        let mut models = alloc::vec::Vec::<Rc<psp_file_formats::Model>>::new();
        let instances = alloc::vec::Vec::<ModelInstance>::new();
        let model = {
            let bytes = include_bytes!("../res/ferris3d_v1.0.psp");
            let model: psp_file_formats::Model = postcard::from_bytes(bytes).unwrap();
            model
        };
        models.push(Rc::new(model));


        Self {
            models,
            instances,
            gizmo: Gizmo::new().ok(),
            timer: Timer::new(),
        }
    }

    fn setup(&mut self)
    {
        let mut instance1 = ModelInstance::from(self.models[0].clone());
        instance1.position = [0.0,-2.0, -1.0];
        self.instances.push(instance1);
        let mut instance2 = ModelInstance::from(self.models[0].clone());
        instance2.position=[-2.0, -2.0, -1.0];
        self.instances.push(instance2);
        let mut instance3 = ModelInstance::from(self.models[0].clone());
        instance3.position=[2.0, -2.0, -1.0];
        self.instances.push(instance3);

        let mut instance4 = ModelInstance::from(self.models[0].clone());
        instance4.position = [0.0,-2.0, -2.0];
        self.instances.push(instance4);
        let mut instance5 = ModelInstance::from(self.models[0].clone());
        instance5.position=[-2.0, -2.0, -2.0];
        self.instances.push(instance5);
        let mut instance6 = ModelInstance::from(self.models[0].clone());
        instance6.position=[2.0, -2.0, -2.0];
        self.instances.push(instance6);

        let mut instance7 = ModelInstance::from(self.models[0].clone());
        instance7.position = [0.0, -2.0, -3.0];
        self.instances.push(instance7);
        let mut instance8 = ModelInstance::from(self.models[0].clone());
        instance8.position=[-2.0, -2.0, -3.0];
        self.instances.push(instance8);
        let mut instance9 = ModelInstance::from(self.models[0].clone());
        instance9.position=[2.0, -2.0, -3.0];
        self.instances.push(instance9);

        let mut instance10 = ModelInstance::from(self.models[0].clone());
        instance10.position = [0.0, -2.0, -4.0];
        self.instances.push(instance10);
        let mut instance11 = ModelInstance::from(self.models[0].clone());
        instance11.position=[-2.0, -2.0, -4.0];
        self.instances.push(instance11);
        let mut instance12 = ModelInstance::from(self.models[0].clone());
        instance12.position=[2.0, -2.0, -4.0];
        self.instances.push(instance12);
    }

    fn draw(&mut self) 
    {
        let delta = self.timer.delta_f32();
        for instance in &mut self.instances {
            let y_rot = delta;
            instance.rotation = [0.0, instance.rotation[1] + y_rot, 0.0];
            instance.draw();
        }
        if let Some(gizmo) = &mut self.gizmo {
            gizmo.draw(delta);
        }
        self.timer.step()
    }
}

impl Default for World {
    fn default() -> Self {
        Self::new()
    }
}

fn psp_main() {
    psp::enable_home_button();

    let mut buffers = GraphicsBuffer::default();
    init_graphics(&mut buffers);

    let mut world = World::new();
    world.setup();

    loop {
        draw_frame(&mut buffers, &mut world);
    }

    // sceGuTerm();
}

fn draw_frame(buffer: &mut GraphicsBuffer, world: &mut World) 
{
    unsafe {
        sceGuStart(
            GuContextType::Direct,
            buffer.display_list.0.as_mut_ptr() as _,
        );
        sceGuClear(ClearBuffer::all());
    };

    world.draw();

    unsafe {
        sceGuFinish();
        sceGuSync(GuSyncMode::Finish, GuSyncBehavior::Wait);

        sceDisplayWaitVblankStart();
        sceGuSwapBuffers();
    }
}

fn init_graphics(buffer: &mut GraphicsBuffer) {
    unsafe {
        sceGumLoadIdentity();

        sceGumMatrixMode(MatrixMode::Projection);
        sceGumLoadIdentity();
        sceGumPerspective(90.0, SCR_ASPECT, 0.1, 10000.0);

        sceGumMatrixMode(MatrixMode::View);
        sceGumLoadIdentity();

        // rotate camera 45 degrees
        sceGumRotateX(45.0f32.to_radians());
        
        sceGumMatrixMode(MatrixMode::Model);
        sceGumLoadIdentity();

        sceGuInit();
        sceGuStart(
            GuContextType::Direct,
            buffer.display_list.0.as_mut_ptr() as _,
        );

        sceGuDrawBuffer(DisplayPixelFormat::Psm8888, VRAM_BP32_0, VRAM_WIDTH);
        sceGuDispBuffer(SCR_WIDTH, SCR_HEIGHT, VRAM_BP32_1, VRAM_WIDTH);
        sceGuDepthBuffer(VRAM_BP32_2, VRAM_WIDTH);

        sceGuOffset(SCR_OFFSETX, SCR_OFFSETY);
        sceGuViewport(2048, 2048, SCR_WIDTH, SCR_HEIGHT);
        sceGuDepthRange(65535, 0);
        sceGuEnable(GuState::ClipPlanes);

        sceGuScissor(0, 0, SCR_WIDTH, SCR_HEIGHT);
        sceGuEnable(GuState::ScissorTest);

        sceGuDepthFunc(DepthFunc::GreaterOrEqual);
        sceGuEnable(GuState::DepthTest);

        sceGuTexFilter(TextureFilter::Nearest, TextureFilter::Nearest);
        sceGuTexFunc(TextureEffect::Modulate, TextureColorComponent::Rgba);
        sceGuTexWrap(GuTexWrapMode::Repeat, GuTexWrapMode::Repeat);
        sceGuDisable(GuState::Texture2D);

        sceGuStencilFunc(StencilFunc::Equal, 0x00, 0xff);
        sceGuStencilOp(
            StencilOperation::Keep,
            StencilOperation::Keep,
            StencilOperation::Incr,
        );

        sceGuFrontFace(FrontFaceDirection::CounterClockwise);
        sceGuEnable(GuState::CullFace);

        sceGuAlphaFunc(AlphaFunc::Greater, 0x00, 0xff);
        sceGuEnable(GuState::AlphaTest);

        sceGuBlendFunc(
            BlendOp::Add,
            BlendFactor::SrcAlpha,
            BlendFactor::OneMinusSrcAlpha,
            0,
            0,
        );
        sceGuEnable(GuState::Blend);

        sceGuAmbient(0xffffffff);
        sceGuModelColor(0x00000000, 0xffffffff, 0xffffffff, 0xffffffff);
        sceGuShadeModel(ShadingModel::Smooth);
        sceGuEnable(GuState::Lighting);

        sceGuClearColor(0x007ac980);
        sceGuClearDepth(0);
        sceGuClearStencil(0);

        sceGuFinish();
        sceGuSync(GuSyncMode::Finish, GuSyncBehavior::Wait);

        sceGuDisplay(true);
    }
}
