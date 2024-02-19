use crate::domains::samplegrids::samplegrid2d::SampleGrid2d;

use super::samplestar::PathStoreT;

pub struct SampleStarStats<N> {
    stat_store: Box<[(String, f32)]>,
    path_stats: Box<[Box<dyn Fn(&SampleGrid2d, &N) -> f32 + Send + Sync>]>,
    step_stats: Box<[Box<dyn Fn(&PathStoreT<N>, &Vec<(usize, usize)>) -> f32 + Send + Sync>]>, 
}

impl<N> SampleStarStats<N> {
    pub(crate) fn new(
        path_stats: Vec<(String, Box<dyn Fn(&SampleGrid2d, &N) -> f32 + Send + Sync>)>,
        step_stats: Vec<(String, Box<dyn Fn(&PathStoreT<N>, &Vec<(usize, usize)>) -> f32 + Send + Sync>)>,
    ) -> Self {
        let mut stat_store = vec![
            ("Paths".to_string(), 0.0),
            ("Exp".to_string(), 0.0), 
            ("AVG Len".to_string(), 0.0)
        ];
        stat_store.extend(path_stats.iter().map(|(name, _)| (name.clone(), 0.0)));
        stat_store.extend(step_stats.iter().map(|(name, _)| (name.clone(), 0.0)));
        Self {
            stat_store: stat_store.into_boxed_slice(),
            path_stats: path_stats.into_iter().map(|(_, x)| x).collect(),
            step_stats: step_stats.into_iter().map(|(_, x)| x).collect(),
        }
    }

    pub(crate) fn add(&mut self, index: usize, val: f32) {
        self.stat_store[index].1 += val;
    }

    pub(crate) fn run_path_stats(&mut self, grid: &SampleGrid2d, path: &Vec<N>) {
        for (i, f) in self.path_stats.iter().enumerate() {
            for node in path {
                self.stat_store[3 + i].1 += f(grid, node) / path.len() as f32;
            }
        }
    }

    pub(crate) fn collate_path_stats(&mut self, epochs: usize) {
        for stat in self.stat_store[1..self.path_stats.len() + 3].iter_mut() {
            stat.1 /= epochs as f32;
        }
    }

    pub(crate) fn run_step_stats(&mut self, path_store: &PathStoreT<N>, adj: &Vec<(usize, usize)>) {
        for (i, f) in self.step_stats.iter().enumerate() {
            self.stat_store[3 + self.path_stats.len() + i].1 += f(path_store, adj);
        }
    }

    pub(crate) fn clear(&mut self) {
        for stat in self.stat_store.iter_mut() {
            stat.1 = 0.0;
        }
    }

    pub fn get_stats(&self) -> Vec<String>{
        self.stat_store.iter().map(|(name, val)| format!("{}: {:.2}", name, val)).collect::<Vec<_>>()
    }
}