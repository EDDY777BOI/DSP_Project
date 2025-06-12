function [filtered_data] = loadAndFilterTSV(filename,fc, fs)
  % Read .tsv and filter data
  data = readtable(filename, 'FileType', 'text', 'Delimiter', '\t');
  dt = 1/fs; % time step 
  [b,a] = butter(2,fc/(fs/2)); % butterworth filter applied
  filtered_data = data;
  for i=1:width(data)
    filtered_data{:,i} = filtfilt(b,a,data{:,i});
  end
end

