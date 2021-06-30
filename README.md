# too_many_dimensions
Toy to mess around with flow fields on the gpu.  

<img width="320px" src="https://user-images.githubusercontent.com/34073738/123915518-34b2c000-d946-11eb-8b8b-17e610b7f0cf.gif">

## Info

I wrote the majority of this code a while back and though it would be fun to share.  
A lot of it is super messy however, so don't go using at a benchmark on how to program something like this.  

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
These logs are compressed after the first two runs and logs more than 15 runs ago are deleted.
