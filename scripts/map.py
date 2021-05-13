import msgpack
import struct
import numpy as np
from collections import namedtuple

SemanticBKIOctoMap = namedtuple("SemanticBKIOctoMap", ['resolution', 'block_depth', 'blocks'])
SEMANTIC_OCTREE_NODE_MSGPACK_EXT_TYPE = 10

class SBKIOctoMapNode:
    def __init__(self) -> None:
        self.data = []

    def __hash__(self) -> int:
        return hash(self.data.tostring())

    def thash(self, tol=1e-9):
        return hash((self.data / tol).astype(int).tostring())

    def is_classified(self) -> bool:
        min_score = np.min(self.data)
        return np.any(self.data != min_score)

    def __str__(self) -> str:
        return str(self.data)

class SBKIOctoMapBlock:
    def __init__(self):
        self.nodes = []

    def __hash__(self) -> int:
        return hash(tuple(hash(n) for n in self.nodes))

    def thash(self, tol=1e-9) -> int:
        return hash(tuple(n.thash(tol) for n in self.nodes))

    def __str__(self) -> str:
        return '[' + '\n'.join([(str(n) if n.is_classified() else '<>') for n in self.nodes]) + ']'

class SBKIOctoMap:
    def __init__(self):
        self.resolution = None
        self.block_depth = None
        self.num_class = 0
        self.blocks = {}

    @classmethod
    def load(cls, map_path):
        m = cls()
        with open(map_path, "rb") as fin:
            data = msgpack.unpack(fin, strict_map_key=False)
        m.resolution, m.block_depth, m.num_class, blocks = data

        for k, nodes in blocks.items():
            octo_block = SBKIOctoMapBlock()
            for n in nodes:
                octo_node = SBKIOctoMapNode()
                assert n.code == SEMANTIC_OCTREE_NODE_MSGPACK_EXT_TYPE
                octo_node.data = np.frombuffer(n.data, dtype='>f4')
                octo_block.nodes.append(octo_node)
            m.blocks[k] = octo_block
        return m

    def __hash__(self) -> int:
        blocks_hash = hash(frozenset((k, v) for k, v in self.blocks.items()))
        return hash((self.resolution, self.block_depth, self.num_class, blocks_hash))

    def thash(self, tol=1e-8) -> int: # hash with tolerance
        assert tol > 0
        blocks_hash = hash(frozenset((k, v.thash(tol)) for k, v in self.blocks.items()))
        return hash((self.resolution, self.block_depth, self.num_class, blocks_hash))

def compare_maps(m1, m2, tol=None):
    if tol is None:
        h1, h2 = hash(m1), hash(m2)
    else:
        h1, h2 = m1.thash(tol), m2.thash(tol)

    if h1 == h2:
        print("Maps are equal.")
        return
    else:
        print("Maps are not equal!!! (by hash)")

    if m1.resolution != m2.resolution:
        print("> resolution:", m1.resolution, m2.resolution)
        return
    if m1.block_depth != m2.block_depth:
        print("> block_depth:", m1.block_depth, m2.block_depth)
        return
    if len(m1.blocks) != len(m2.blocks):
        print("> Block counts:", len(m1.blocks), len(m2.blocks))
        return

    bk1 = hash(frozenset(m1.blocks.keys()))
    bk2 = hash(frozenset(m2.blocks.keys()))
    if bk1 != bk2:
        print("> Block keys are different!!")
        return

    if tol is None:
        bh1 = {k: hash(v) for k, v in m1.blocks.items()}
        bh2 = {k: hash(v) for k, v in m2.blocks.items()}
    else:
        bh1 = {k: v.thash(tol) for k, v in m1.blocks.items()}
        bh2 = {k: v.thash(tol) for k, v in m2.blocks.items()}
    mismatch = [k for k, v in bh1.items() if bh2[k] != v]
    mismatch_items = []
    for k in mismatch:
        for i, (b, bref) in enumerate(zip(m.blocks[k].nodes, mref.blocks[k].nodes)):
            if (hash(b) == hash(bref) if tol is None else b.thash(tol) == bref.thash(tol)):
                continue

            idiff, = np.where(np.abs(b.data - bref.data) > tol)
            if len(idiff) > 0:
                for j, d, dref in zip(idiff, b.data[idiff], bref.data[idiff]):
                    mismatch_items.append((k, i, j, d, dref, abs(d-dref) / dref))

    if not mismatch_items:
        print("Maps are actually equal. (by comparison)")
        return

    print("> Top 10 mismatches:")
    mismatch_items.sort(key = lambda item: -item[-1]) # sort by error rate
    for k, i, j, d, dref, r in mismatch_items[:10]:
        print(f"  Block {k}, node {i}, {j}th data: {d} <-> {dref} (err: {np.format_float_scientific(r, precision=3)})")
    print("> Mismatch rate", len(mismatch), '/', len(m.blocks))

if __name__ == "__main__":
    # import fire
    # fire.Fire(dict(hash=map_hash))
    m = SBKIOctoMap.load("map_dump.bin")
    mref = SBKIOctoMap.load("/home/jacobz/Coding/ws/src/map_dump_ref.bin")
    compare_maps(m, mref, 1e-4)
