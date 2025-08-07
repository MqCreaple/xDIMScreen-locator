use std::{env, fs::{self, File}, io::{Read, Write}, net::TcpStream, path::Path, time::SystemTime};

use xDIMScreen_locator::net::packet::ObjectLocationPacket;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
struct Sample {
    timestamp_sent: u128,
    timestamp_recv: u128,
    num_objects: usize,
}

fn read_line(stream: &mut impl Read) -> Result<String, std::io::Error> {
    let mut buf = [0u8; 1024];
    let mut len = 0;
    stream.read(&mut buf[len..len+1])?;
    while len < 1024 && buf[len] != b'\n' {
        len += 1;
        stream.read(&mut buf[len..len+1])?;
    }
    Ok(String::from_utf8_lossy(&buf[0..len]).into_owned())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::new()
        .filter_level(log::LevelFilter::Info)
        .try_init()?;

    // create the directory to put benchmark results
    let benchmark_dir = Path::new(&env::current_dir()?).join("benchmark-results");
    if benchmark_dir.exists() && !benchmark_dir.is_dir() {
        return Err("There is already a file named \"benchmark-results\". Please remove it before running the benchmarker.".into());
    } else if !benchmark_dir.exists() {
        fs::create_dir(benchmark_dir.clone())?;
    }

    const N_SAMPLES: usize = 1000;
    let mut samples = [Sample::default(); N_SAMPLES];

    const PORT: u16 = 30002;
    let mut stream = TcpStream::connect(format!("127.0.0.1:{}", PORT))?;
    log::info!("Connected to address 127.0.0.1:{}", PORT);

    let mut index = 0;
    while index < N_SAMPLES {
        let line = read_line(&mut stream)?;
        let packet: ObjectLocationPacket = serde_json::from_str(&line)?;
        let now = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH)?.as_millis();
        if index == 0 || samples[index - 1].timestamp_sent != packet.time {
            samples[index] = Sample {
                timestamp_sent: packet.time,
                timestamp_recv: now,
                num_objects: 1,
            };
            log::info!("Sample {}", index);
            index += 1;
        } else {
            // this packet is sent at the same timestamp as the last packet
            // simply add the object count by 1
            samples[index - 1].num_objects += 1;
        }
    }

    // save the result to CSV
    log::info!("Finished recording samples. Saving to CSV...");
    let mut csv_file = File::create(benchmark_dir.join("samples.csv"))?;
    writeln!(csv_file, "timestamp sent,timestamp received,number of objects")?;
    for sample in samples {
        writeln!(csv_file, "{},{},{}", sample.timestamp_sent, sample.timestamp_recv, sample.num_objects)?;
    }
    log::info!("Finished saving to CSV file.");

    // statistics
    let total_send_recv_delay = samples.iter()
        .map(|sample| sample.timestamp_recv - sample.timestamp_sent)
        .sum::<u128>() as f64;
    let mean_send_recv_delay = total_send_recv_delay / (N_SAMPLES as f64);
    log::info!("Average delay between timestamp is: {:.2} ms", mean_send_recv_delay);
    let sum_square_send_recv_delay = samples.iter()
        .map(|sample| ((sample.timestamp_recv - sample.timestamp_sent) as f64 - mean_send_recv_delay))
        .map(|x| x * x)
        .sum::<f64>();
    let std_send_recv_delay = f64::sqrt(sum_square_send_recv_delay / (N_SAMPLES as f64 - 1.0));
    log::info!("Standard deviation of delay between timestamp is: {:.2} ms", std_send_recv_delay);
    Ok(())
}