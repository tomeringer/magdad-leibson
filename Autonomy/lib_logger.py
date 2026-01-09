import csv, os, time

class CSVMetricLogger:
    def __init__(self, path, fieldnames):
        self.path = path
        self.fieldnames = fieldnames
        new_file = not os.path.exists(path)

        self.f = open(path, "a", newline="")
        self.w = csv.DictWriter(self.f, fieldnames=fieldnames, extrasaction="ignore")
        if new_file:
            self.w.writeheader()

    def log(self, **row):
        # add timestamp automatically
        row.setdefault("t", time.time())
        self.w.writerow(row)
        self.f.flush()  # so you can tail/plot while running

    def close(self):
        self.f.close()

# inside your loop:
# metrics.log(frame=i, dt_ms=dt*1000, detected=int(found), score=conf, area=blob_area)