# too_many_dimensions
Toy to mess around with flow fields on the gpu.  

## Video
https://user-images.githubusercontent.com/34073738/123766954-bcd48f00-d88c-11eb-8386-34a5ee2376c9.mp4

## Ramblings
I wrote the majority of this code a while ago and in turn, it's not my best work.  
In spite of that, I'm keeping this here because I think flow fields are fun, and maybe someone can benefit from what I've done.  
This also uses a very early version of my game (sorta) framework/engine, which again, makes this a fun thing to look back to.  

Made using [wpgu](https://crates.io/crates/wgpu)

## Run
run with `cargo +nightly run --release`  

## Controls
`wasd` to move  
`shift` to move faster  
`control` to move slower  
`scroll` to zoom  
`space` to pause  
`f11` or `alt + enter` to fullscreen  
`escape` to close  

## Config
All of the settings are configurable in `config/config.toml`  
Also, some of the settings don't do anything because I was planning to do something with them but never got around to it.  

## Logging
Program logs are recorded in `log/`
