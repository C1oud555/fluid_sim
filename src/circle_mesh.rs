pub struct CircleMesh {
    pub vertices: Vec<[f32; 2]>,
    pub indices: Vec<u16>,
}

impl CircleMesh {
    pub fn new(segments: usize) -> Self {
        let mut vertices = vec![[0.0, 0.0]]; // 中心点
        let mut indices = Vec::new();

        // 生成外圈顶点
        for i in 0..=segments {
            let angle = 2.0 * std::f32::consts::PI * (i as f32) / (segments as f32);
            vertices.push([angle.cos(), angle.sin()]);
        }

        // 生成三角形索引 (中心点 + 相邻外圈点)
        for i in 1..=segments {
            indices.extend_from_slice(&[0, i as u16, ((i % segments) + 1) as u16]);
        }

        Self { vertices, indices }
    }
}
