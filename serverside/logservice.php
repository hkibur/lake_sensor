<?php
$folder_path = "sensor_logs/";
$config_path = "sensor_logs/config.txt";
# ms delta (unsigned long), sample average (double)
$sample_average_error_len = 1;
$sample_average_delta_len = 8;
$sample_average_len = 8;
$mac_addr_len = 6;
$device_name_len = 65;
$config_len = 5;
$total_len = $sample_average_error_len + $sample_average_delta_len + $sample_average_len;

$data_raw = file_get_contents("php://input");
$data_len = strlen($data_raw);

if($data_len == 0){
    exit("POST length of 0");
}

$mac_addr = bin2hex(substr($data_raw, 0, $mac_addr_len));
$dev_name = strtok(substr($data_raw, $mac_addr_len, $device_name_len), "\x00");
$filename = $dev_name . "-" . $mac_addr . ".log";
$data_offset = $mac_addr_len + $device_name_len;

$log_reverse_array = array();

if(($data_len - $data_offset) % $total_len == 0){
    $log_handle = fopen($folder_path . $filename, "a");
    $average_time = intval(microtime(true) * 1000);
    for($i = $data_offset; $i < $data_len; $i += $total_len){
        $unpacked_arr = unpack("Cerror/Pdelta_ms/eaverage", $data_raw, $i);
        $log_string = date("Y-m-dTH:i:s", intval($average_time / 1000)) . "." . str_pad(strval($average_time % 10000), 4, "0", STR_PAD_RIGHT) . " | 0x" . str_pad(strtoupper(dechex($unpacked_arr["error"])), 2, "0", STR_PAD_LEFT) . " | " . strval($unpacked_arr["average"]) . "\n";
        $average_time -= $unpacked_arr["delta_ms"];
        array_unshift($log_reverse_array, $log_string);
    }
    foreach($log_reverse_array as $log_string){
        fwrite($log_handle, $log_string);
    }
    fclose($log_handle);
    $ret_handle = fopen($config_path, "r");
    echo hex2bin(fread($ret_handle, $config_len * 2));
    fclose($ret_handle);
}
?>