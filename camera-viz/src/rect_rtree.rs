use crate::message as msg;
use opencv::core::{Point2f, Rect};
use rstar::{primitives::Rectangle, Envelope, Point, PointDistance, RTree, RTreeObject, AABB};

/// An R-Tree storing spatially indexed rectangles.
#[derive(Debug)]
pub struct RectRTree {
    rtree: RTree<Entry>,
}

impl RectRTree {
    /// Finds a rectangle containing the query point.
    pub fn find(&self, point: &Point2f) -> Option<msg::ArcRect> {
        let Point2f { x, y } = *point;
        self.rtree
            .locate_at_point(&[x, y])
            .map(|entry| entry.rect.clone())
    }
}

impl FromIterator<msg::ArcRect> for RectRTree {
    fn from_iter<T>(iter: T) -> Self
    where
        T: IntoIterator<Item = msg::ArcRect>,
    {
        let vec: Vec<_> = iter.into_iter().map(|rect| Entry { rect }).collect();
        let rtree = RTree::bulk_load(vec);
        Self { rtree }
    }
}

/// An entry of the RectRTree.
#[derive(Debug)]
struct Entry {
    rect: msg::ArcRect,
}

impl RTreeObject for Entry {
    type Envelope = AABB<[f32; 2]>;

    fn envelope(&self) -> Self::Envelope {
        let Rect {
            x,
            y,
            width,
            height,
        } = *self.rect;
        let lt = [
            x as f32 - width as f32 / 2.0,
            y as f32 - height as f32 / 2.0,
        ];
        let rb = [
            x as f32 + width as f32 / 2.0,
            y as f32 + height as f32 / 2.0,
        ];
        AABB::from_corners(lt, rb)
    }
}

impl PointDistance for Entry {
    fn distance_2(
        &self,
        point: &<Self::Envelope as Envelope>::Point,
    ) -> <<Self::Envelope as Envelope>::Point as Point>::Scalar {
        let rect = Rectangle::from_aabb(self.envelope());
        rect.distance_2(point)
    }
}
