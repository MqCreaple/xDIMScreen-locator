use std::io::Write;
use std::net::TcpListener;
use std::sync::{Arc, Condvar, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};

use crate::tag::locator::LocatedObjects;

pub mod packet;

pub fn server_thread_main<'a>(
    port: u16,
    located_objects: Arc<(Mutex<LocatedObjects<'a>>, Condvar)>,
) -> Result<(), Box<dyn std::error::Error>> {
    // open server
    let listener = TcpListener::bind(format!("127.0.0.1:{}", port))?;
    log::info!("Server started at port {}", port);
    let (mut stream, addr) = loop {
        let conn = listener.accept();
        match conn {
            Ok((stream, addr)) => {
                break (stream, addr);
            }
            Err(e) => {
                log::error!("An error occurred at TCP server: {}", e);
            }
        }
    };
    log::info!("Accepted client {}. Connection established.", addr);

    // set up conditional variable
    let mut locked_located_objects = located_objects.0.lock().unwrap();
    let mut last_timestamp = SystemTime::now();
    'main_loop: loop {
        // get all the detected objects
        locked_located_objects = located_objects
            .1
            .wait_while(locked_located_objects, |v| {
                (v.timestamp() == last_timestamp) || v.name_map().is_empty()
            })
            .unwrap();
        last_timestamp = locked_located_objects.timestamp();
        // convert the map to a list of packets
        for (name, location) in locked_located_objects.name_map() {
            let packet = packet::ObjectLocationPacket {
                time: last_timestamp.duration_since(UNIX_EPOCH)?.as_millis(),
                name: name.to_string(),
                transform: location.clone(),
            };
            let serialized = serde_json::to_string(&packet)?;
            let stream_write_result = stream
                .write(serialized.as_bytes())
                .and_then(|_| stream.write(b"\n"));
            match stream_write_result {
                Ok(_) => {}
                Err(e) => {
                    // Error occurred when writing to the stream. Jump out of the loop.
                    log::error!("Error occurred with client {}: {}", addr, e);
                    break 'main_loop;
                }
            }
        }
    }
    Ok(())
}
