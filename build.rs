use anyhow::*;
use glob::glob;
use rayon::prelude::*;
use std::fs::{read_to_string, write};
use std::path::PathBuf;

struct ShaderData {
    src: String,
    src_path: PathBuf,
    spv_path: PathBuf,
    kind: shaderc::ShaderKind,
}
impl ShaderData {
    pub fn load(src_path: PathBuf) -> Result<Self> {
        let extension = src_path
            .extension()
            .context("File has no extension")?
            .to_str()
            .context("Extension cannot be converted to &str")?;

        let kind = match extension {
            "vert" => shaderc::ShaderKind::Vertex,
            "frag" => shaderc::ShaderKind::Fragment,
            "comp" => shaderc::ShaderKind::Compute,
            _ => bail!("Unsupported shader: {}", src_path.display()),
        };

        let src = read_to_string(src_path.clone())?;
        let spv_path = src_path
            .parent()
            .unwrap()
            .parent()
            .unwrap()
            .parent()
            .unwrap()
            .join("spirv")
            .join(
                src_path
                    .file_name()
                    .unwrap()
                    .to_os_string()
                    .to_str()
                    .unwrap(),
            )
            .with_extension(format!("{}.spv", extension));

        Ok(Self {
            src,
            src_path,
            spv_path,
            kind,
        })
    }
}

fn main() -> Result<()> {
    // Collect all shaders recursively within /src/
    let mut shader_paths = Vec::new();
    shader_paths.extend(glob("./src/**/*.vert")?);
    shader_paths.extend(glob("./src/**/*.frag")?);
    shader_paths.extend(glob("./src/**/*.comp")?);

    let shaders = shader_paths
        .into_par_iter()
        .flatten()
        .map(|glob_result| ShaderData::load(glob_result.clone()))
        .collect::<Vec<Result<_>>>()
        .into_iter()
        .collect::<Result<Vec<_>>>()?;

    let mut compiler = shaderc::Compiler::new().context("Unable to create shader compiler")?;
    let mut options = shaderc::CompileOptions::new().context("Unable to create compiler options")?;
    options.set_optimization_level(shaderc::OptimizationLevel::Performance);

    for shader in shaders {
        println!(
            "cargo:rerun-if-changed={}",
            shader.src_path.as_os_str().to_str().unwrap()
        );

        let compiled = compiler.compile_into_spirv(
            &shader.src,
            shader.kind,
            &shader.src_path.to_str().unwrap(),
            "main",
            Some(&options),
        )?;
        write(shader.spv_path, compiled.as_binary_u8())?;
    }

    Ok(())
}
