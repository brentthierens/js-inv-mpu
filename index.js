const { Worker } = require('worker_threads');

let driver =
{
    sampleFreq: 200,
    filterFreq: 20,
    worker: undefined,
    setSampleFreq: function(freq){ this.sampleFreq = freq; },
    setDlpf: function(freq){ this.filterFreq = freq; },
    measure: function(callback){ new Worker("./worker.js", { workerData: { sample: this.sampleFreq, filter: this.filterFreq }}).on('message', callback); }
};

module.exports = driver;